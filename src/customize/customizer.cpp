#include "extractor/node_data_container.hpp"

#include "customizer/cell_customizer.hpp"
#include "customizer/customizer.hpp"
#include "customizer/edge_based_graph.hpp"
#include "customizer/files.hpp"

#include "partitioner/cell_statistics.hpp"
#include "partitioner/cell_storage.hpp"
#include "partitioner/edge_based_graph_reader.hpp"
#include "partitioner/files.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "storage/shared_memory_ownership.hpp"

#include "updater/updater.hpp"

#include "util/exclude_flag.hpp"
#include "util/log.hpp"
#include "util/timing_util.hpp"

#include <boost/assert.hpp>

#include <tbb/global_control.h>

namespace osrm::customizer
{

namespace
{

template <typename Partition, typename CellStorage>
void printUnreachableStatistics(const Partition &partition,
                                const CellStorage &storage,
                                const CellMetric &metric)
{
    util::Log() << "Unreachable nodes statistics per level";

    for (std::size_t level = 1; level < partition.GetNumberOfLevels(); ++level)
    {
        auto num_cells = partition.GetNumberOfCells(level);
        std::size_t invalid_sources = 0;
        std::size_t invalid_destinations = 0;
        for (std::uint32_t cell_id = 0; cell_id < num_cells; ++cell_id)
        {
            const auto &cell = storage.GetCell(metric, level, cell_id);
            for (auto node : cell.GetSourceNodes())
            {
                const auto &weights = cell.GetOutWeight(node);
                invalid_sources +=
                    std::all_of(weights.begin(),
                                weights.end(),
                                [](auto weight) { return weight == INVALID_EDGE_WEIGHT; });
            }
            for (auto node : cell.GetDestinationNodes())
            {
                const auto &weights = cell.GetInWeight(node);
                invalid_destinations +=
                    std::all_of(weights.begin(),
                                weights.end(),
                                [](auto weight) { return weight == INVALID_EDGE_WEIGHT; });
            }
        }

        if (invalid_sources > 0 || invalid_destinations > 0)
        {
            util::Log(logWARNING) << "Level " << level << " unreachable boundary nodes per cell: "
                                  << (invalid_sources / (float)num_cells) << " sources, "
                                  << (invalid_destinations / (float)num_cells) << " destinations";
        }
    }
}

auto LoadAndUpdateEdgeExpandedGraph(const CustomizationConfig &config,
                                    const partitioner::MultiLevelPartition &mlp,
                                    std::vector<EdgeWeight> &node_weights,
                                    std::vector<EdgeDuration> &node_durations,
                                    std::vector<EdgeDistance> &node_distances,
                                    std::uint32_t &connectivity_checksum)
{
    updater::Updater updater(config.updater_config);

    std::vector<extractor::EdgeBasedEdge> edge_based_edge_list;
    EdgeID num_nodes = updater.LoadAndUpdateEdgeExpandedGraph(
        edge_based_edge_list, node_weights, node_durations, connectivity_checksum);

    extractor::files::readEdgeBasedNodeDistances(config.GetPath(".osrm.enw"), node_distances);

    auto directed = partitioner::splitBidirectionalEdges(edge_based_edge_list);

    auto tidied = partitioner::prepareEdgesForUsageInGraph<
        typename partitioner::MultiLevelEdgeBasedGraph::InputEdge>(std::move(directed));

    auto edge_based_graph = partitioner::MultiLevelEdgeBasedGraph(mlp, num_nodes, tidied);

    return edge_based_graph;
}

std::vector<CellMetric> customizeFilteredMetrics(const partitioner::MultiLevelEdgeBasedGraph &graph,
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

int Customizer::Run(const CustomizationConfig &config,
                    const std::vector<std::pair<std::size_t, std::string>> &period_speed_files)
{
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism,
                           config.requested_num_threads);

    const bool multi_period = !period_speed_files.empty();

    TIMER_START(loading_data);

    partitioner::MultiLevelPartition mlp;
    partitioner::files::readPartition(config.GetPath(".osrm.partition"), mlp);

    partitioner::CellStorage storage;
    partitioner::files::readCells(config.GetPath(".osrm.cells"), storage);

    extractor::EdgeBasedNodeDataContainer node_data;
    extractor::files::readNodeData(config.GetPath(".osrm.ebg_nodes"), node_data);

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(config.GetPath(".osrm.properties"), properties);

    TIMER_STOP(loading_data);
    util::Log() << "Loading partition data took " << TIMER_SEC(loading_data) << " seconds";

    // Build the list of (period_index, speed_csv_path) to iterate.
    // Legacy single-period: synthesize one entry from config's speed paths.
    std::vector<std::pair<std::size_t, std::string>> periods;
    if (multi_period)
    {
        periods = period_speed_files;
        util::Log() << "Multi-period customize: " << periods.size() << " periods";
    }
    else
    {
        // Legacy: use first speed lookup path from config, period 0
        std::string speed_path;
        if (!config.updater_config.segment_speed_lookup_paths.empty())
            speed_path = config.updater_config.segment_speed_lookup_paths.front();
        periods.emplace_back(0, speed_path);
    }

    // Customize each period, accumulate metrics
    std::vector<std::pair<std::size_t, std::vector<CellMetric>>> all_period_metrics;
    all_period_metrics.reserve(periods.size());

    std::uint32_t connectivity_checksum = 0;
    std::vector<EdgeWeight> last_node_weights;
    std::vector<EdgeDuration> last_node_durations;
    std::vector<EdgeDistance> last_node_distances;
    partitioner::MultiLevelEdgeBasedGraph last_graph;

    for (const auto &[period_index, speed_csv_path] : periods)
    {
        TIMER_START(period_customize);

        auto period_config = config;
        if (!speed_csv_path.empty())
            period_config.updater_config.segment_speed_lookup_paths = {speed_csv_path};

        std::vector<EdgeWeight> node_weights;
        std::vector<EdgeDuration> node_durations;
        std::vector<EdgeDistance> node_distances;
        auto graph = LoadAndUpdateEdgeExpandedGraph(
            period_config, mlp, node_weights, node_durations, node_distances, connectivity_checksum);
        BOOST_ASSERT(graph.GetNumberOfNodes() == node_weights.size());
        std::for_each(node_weights.begin(),
                      node_weights.end(),
                      [](auto &w) { w &= EdgeWeight{0x7fffffff}; });

        if (all_period_metrics.empty())
        {
            util::Log() << "Loaded edge based graph: " << graph.GetNumberOfEdges() << " edges, "
                        << graph.GetNumberOfNodes() << " nodes";
            partitioner::printCellStatistics(mlp, storage);
        }

        auto filter =
            util::excludeFlagsToNodeFilter(graph.GetNumberOfNodes(), node_data, properties);
        auto metrics = customizeFilteredMetrics(graph, storage, CellCustomizer{mlp}, filter);

        TIMER_STOP(period_customize);
        util::Log() << "Period " << period_index << " customization took "
                    << TIMER_SEC(period_customize) << " seconds";

        for (const auto &metric : metrics)
        {
            printUnreachableStatistics(mlp, storage, metric);
        }

        all_period_metrics.emplace_back(period_index, std::move(metrics));

        last_node_weights = std::move(node_weights);
        last_node_durations = std::move(node_durations);
        last_node_distances = std::move(node_distances);
        last_graph = std::move(graph);
    }

    // Write cell metrics
    TIMER_START(writing_mld_data);
    if (multi_period)
    {
        files::writeMultiPeriodCellMetrics(
            config.GetPath(".osrm.cell_metrics"), properties.GetWeightName(), all_period_metrics);
    }
    else
    {
        // Legacy format for backward compat with non-period-aware OSRM
        std::unordered_map<std::string, std::vector<CellMetric>> metric_exclude_classes = {
            {properties.GetWeightName(), std::move(all_period_metrics[0].second)},
        };
        files::writeCellMetrics(config.GetPath(".osrm.cell_metrics"), metric_exclude_classes);
    }
    TIMER_STOP(writing_mld_data);
    util::Log() << "MLD customization writing took " << TIMER_SEC(writing_mld_data) << " seconds";

    // Write graph (topology is same across periods, edge weights from last)
    TIMER_START(writing_graph);
    MultiLevelEdgeBasedGraph shaved_graph{std::move(last_graph),
                                          std::move(last_node_weights),
                                          std::move(last_node_durations),
                                          std::move(last_node_distances)};
    customizer::files::writeGraph(
        config.GetPath(".osrm.mldgr"), shaved_graph, connectivity_checksum);
    TIMER_STOP(writing_graph);
    util::Log() << "Graph writing took " << TIMER_SEC(writing_graph) << " seconds";

    return 0;
}

} // namespace osrm::customizer
