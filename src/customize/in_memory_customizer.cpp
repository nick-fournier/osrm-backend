#include "customizer/in_memory_customizer.hpp"

#include "customizer/cell_customizer.hpp"
#include "customizer/files.hpp"

#include "extractor/node_data_container.hpp"

#include "partitioner/cell_storage.hpp"
#include "partitioner/edge_based_graph_reader.hpp"
#include "partitioner/files.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "updater/updater.hpp"

#include "util/exclude_flag.hpp"
#include "util/log.hpp"
#include "util/timing_util.hpp"

#include <tbb/global_control.h>

namespace osrm::customizer
{

namespace
{
/// Run CellCustomizer for each node filter, return vector of CellMetric.
std::vector<CellMetric>
customizeAllFilters(const partitioner::MultiLevelEdgeBasedGraph &graph,
                    const partitioner::CellStorage &storage,
                    const CellCustomizer &customizer,
                    const std::vector<std::vector<bool>> &node_filters)
{
    std::vector<CellMetric> metrics;
    metrics.reserve(node_filters.size());
    for (const auto &filter : node_filters)
    {
        auto metric = storage.MakeMetric();
        customizer.Customize(graph, storage, filter, metric);
        metrics.push_back(std::move(metric));
    }
    return metrics;
}
} // namespace

void InMemoryCustomizer::Initialize(const CustomizationConfig &config)
{
    config_ = config;

    TIMER_START(init);

    // Load partition and cell storage (cached for all subsequent calls)
    partitioner::files::readPartition(config.GetPath(".osrm.partition"), mlp_);
    partitioner::files::readCells(config.GetPath(".osrm.cells"), storage_);

    // Load node data and properties for exclude-flag filters
    extractor::EdgeBasedNodeDataContainer node_data;
    extractor::files::readNodeData(config.GetPath(".osrm.ebg_nodes"), node_data);

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(config.GetPath(".osrm.properties"), properties);

    // Build the first graph via the updater (full path: CSV → edges → graph)
    updater::Updater updater(config.updater_config);
    std::vector<extractor::EdgeBasedEdge> edge_based_edge_list;
    std::vector<EdgeWeight> node_weights;
    std::vector<EdgeDuration> node_durations;
    num_nodes_ = updater.LoadAndUpdateEdgeExpandedGraph(
        edge_based_edge_list, node_weights, node_durations, connectivity_checksum_);

    std::vector<EdgeDistance> node_distances;
    extractor::files::readEdgeBasedNodeDistances(config.GetPath(".osrm.enw"), node_distances);

    // Mask high bit (used as flag by OSRM)
    for (auto &w : node_weights)
        w &= EdgeWeight{0x7fffffff};

    // Build node filters (cached — only depends on topology)
    node_filters_ = util::excludeFlagsToNodeFilter(node_weights.size(), node_data, properties);

    // Build the MLD graph (split → sort → construct)
    auto directed = partitioner::splitBidirectionalEdges(edge_based_edge_list);
    auto tidied = partitioner::prepareEdgesForUsageInGraph<
        typename partitioner::MultiLevelEdgeBasedGraph::InputEdge>(std::move(directed));
    graph_ = partitioner::MultiLevelEdgeBasedGraph(mlp_, num_nodes_, tidied);

    // Run initial cell customization
    auto metrics = customizeAllFilters(graph_, storage_, CellCustomizer{mlp_}, node_filters_);

    TIMER_STOP(init);
    util::Log() << "InMemoryCustomizer initialized in " << TIMER_SEC(init) << "s"
                << " (" << graph_.GetNumberOfEdges() << " edges, "
                << graph_.GetNumberOfNodes() << " nodes)";

    initialized_ = true;
}

double InMemoryCustomizer::Recustomize(const std::string &speed_csv_path)
{
    BOOST_ASSERT_MSG(initialized_, "Must call Initialize() before Recustomize()");

    tbb::global_control gc(tbb::global_control::max_allowed_parallelism,
                           config_.requested_num_threads);

    // Re-run the updater with new speed CSV → fresh edge weights
    auto updater_config = config_.updater_config;
    if (!speed_csv_path.empty())
        updater_config.segment_speed_lookup_paths = {speed_csv_path};
    else
        updater_config.segment_speed_lookup_paths.clear();
    updater::Updater updater(updater_config);

    std::vector<extractor::EdgeBasedEdge> edge_based_edge_list;
    std::vector<EdgeWeight> node_weights;
    std::vector<EdgeDuration> node_durations;
    std::uint32_t checksum = 0;
    updater.LoadAndUpdateEdgeExpandedGraph(
        edge_based_edge_list, node_weights, node_durations, checksum);

    std::vector<EdgeDistance> node_distances;
    extractor::files::readEdgeBasedNodeDistances(config_.GetPath(".osrm.enw"), node_distances);

    for (auto &w : node_weights)
        w &= EdgeWeight{0x7fffffff};

    // Rebuild graph with new weights (topology unchanged, weights differ)
    auto directed = partitioner::splitBidirectionalEdges(edge_based_edge_list);
    auto tidied = partitioner::prepareEdgesForUsageInGraph<
        typename partitioner::MultiLevelEdgeBasedGraph::InputEdge>(std::move(directed));
    graph_ = partitioner::MultiLevelEdgeBasedGraph(mlp_, num_nodes_, tidied);

    // Run cell Dijkstra (the expensive part)
    TIMER_START(cell_customize);
    auto metrics = customizeAllFilters(graph_, storage_, CellCustomizer{mlp_}, node_filters_);
    TIMER_STOP(cell_customize);
    double cell_time = TIMER_SEC(cell_customize);

    // Store latest metrics for callers to retrieve
    latest_metrics_ = std::move(metrics);

    return cell_time;
}

} // namespace osrm::customizer
