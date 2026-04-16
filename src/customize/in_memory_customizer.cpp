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

#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

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

/// Like prepareEdgesForUsageInGraph but never merges forward/backward edges.
/// This keeps a stable edge count so weights can be patched in-place.
template <typename OutputEdgeT>
std::vector<OutputEdgeT> prepareEdgesNoMerge(std::vector<extractor::EdgeBasedEdge> edges)
{
    // Same sort: (source, target, forward-first, weight)
    tbb::parallel_sort(begin(edges), end(edges),
        [](const auto &lhs, const auto &rhs)
        {
            return std::tie(lhs.source, lhs.target, rhs.data.forward, lhs.data.weight) <
                   std::tie(rhs.source, rhs.target, lhs.data.forward, rhs.data.weight);
        });

    std::vector<OutputEdgeT> output;
    output.reserve(edges.size());

    for (auto it = edges.begin(); it != edges.end();)
    {
        const NodeID source = it->source;
        const NodeID target = it->target;

        auto end_interval = std::find_if_not(it, edges.end(),
            [source, target](const auto &e)
            { return std::tie(e.source, e.target) == std::tie(source, target); });

        // Remove self-loops
        if (source == target) { it = end_interval; continue; }

        // Find boundary between forward and backward edges
        auto first_bwd = std::find_if(it, end_interval,
            [](const auto &e) { return !e.data.forward && e.data.backward; });

        // Keep first (smallest-weight) forward edge
        if (it != first_bwd)
            output.push_back(OutputEdgeT{source, target, it->data});

        // Keep first (smallest-weight) backward edge — always separate
        if (first_bwd != end_interval)
            output.push_back(OutputEdgeT{source, target, first_bwd->data});

        it = end_interval;
    }

    return output;
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
    // skip_geometry_write: keep extraction files immutable
    auto init_updater_config = config.updater_config;
    init_updater_config.skip_geometry_write = true;
    updater::Updater updater(init_updater_config);
    std::vector<extractor::EdgeBasedEdge> edge_based_edge_list;
    std::vector<EdgeWeight> node_weights;
    std::vector<EdgeDuration> node_durations;
    num_nodes_ = updater.LoadAndUpdateEdgeExpandedGraph(
        edge_based_edge_list, node_weights, node_durations, connectivity_checksum_);

    // Mask high bit (used as flag by OSRM)
    for (auto &w : node_weights)
        w &= EdgeWeight{0x7fffffff};

    // Build node filters (cached — only depends on topology)
    node_filters_ = util::excludeFlagsToNodeFilter(node_weights.size(), node_data, properties);

    // Build the MLD graph using no-merge split to keep stable topology
    auto directed = partitioner::splitBidirectionalEdges(edge_based_edge_list);
    auto tidied = prepareEdgesNoMerge<
        typename partitioner::MultiLevelEdgeBasedGraph::InputEdge>(std::move(directed));
    graph_ = partitioner::MultiLevelEdgeBasedGraph(mlp_, num_nodes_, tidied);

    // Build edge mapping: original edge index → graph edge ID (per direction)
    // The graph stores edges in the same order as `tidied`.  We re-split
    // (without sorting) to build a turn_id→original_index lookup, then scan
    // graph edges to populate the mapping.
    {
        // turn_id → original index  (turn_id is unique per edge)
        std::unordered_map<NodeID, std::size_t> turn_to_orig;
        turn_to_orig.reserve(edge_based_edge_list.size());
        for (std::size_t i = 0; i < edge_based_edge_list.size(); ++i)
            turn_to_orig[edge_based_edge_list[i].data.turn_id] = i;

        edge_to_graph_fwd_.assign(edge_based_edge_list.size(), SPECIAL_EDGEID);
        edge_to_graph_rev_.assign(edge_based_edge_list.size(), SPECIAL_EDGEID);

        for (NodeID n = 0; n < graph_.GetNumberOfNodes(); ++n)
        {
            for (auto e : graph_.GetAdjacentEdgeRange(n))
            {
                const auto &data = graph_.GetEdgeData(e);
                auto it = turn_to_orig.find(data.turn_id);
                if (it == turn_to_orig.end())
                    continue;
                auto orig_idx = it->second;
                const auto &orig = edge_based_edge_list[orig_idx];

                // Forward split: graph source == original source
                // Reverse split: graph source == original target
                if (n == orig.source)
                    edge_to_graph_fwd_[orig_idx] = e;
                else if (n == orig.target)
                    edge_to_graph_rev_[orig_idx] = e;
            }
        }
    }

    // Run initial cell customization (all filters — cold start)
    latest_metrics_ = customizeAllFilters(graph_, storage_, CellCustomizer{mlp_}, node_filters_);

    TIMER_STOP(init);
    util::Log() << "InMemoryCustomizer initialized in " << TIMER_SEC(init) << "s"
                << " (" << graph_.GetNumberOfEdges() << " edges, "
                << graph_.GetNumberOfNodes() << " nodes)";

    initialized_ = true;
}

double InMemoryCustomizer::Recustomize(const std::string &speed_csv_path,
                                       const std::vector<std::size_t> &filter_indices)
{
    BOOST_ASSERT_MSG(initialized_, "Must call Initialize() before Recustomize()");

    tbb::global_control gc(tbb::global_control::max_allowed_parallelism,
                           config_.requested_num_threads);

    // Re-run the updater with new speed CSV → fresh edge weights
    auto updater_config = config_.updater_config;
    updater_config.skip_geometry_write = true;
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

    for (auto &w : node_weights)
        w &= EdgeWeight{0x7fffffff};

    // Patch edge weights in-place on the cached graph (skip full rebuild)
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, edge_based_edge_list.size()),
        [&](const auto &range)
        {
            for (auto i = range.begin(); i < range.end(); ++i)
            {
                const auto &edge = edge_based_edge_list[i];
                const auto new_weight = std::max(edge.data.weight, EdgeWeight{1});
                const auto new_duration = to_alias<EdgeDuration>(edge.data.duration);
                const auto new_distance = edge.data.distance;

                if (auto fwd = edge_to_graph_fwd_[i]; fwd != SPECIAL_EDGEID)
                {
                    auto &data = graph_.GetEdgeData(fwd);
                    data.weight = new_weight;
                    data.duration = from_alias<EdgeDuration::value_type>(new_duration);
                    data.distance = new_distance;
                }
                if (auto rev = edge_to_graph_rev_[i]; rev != SPECIAL_EDGEID)
                {
                    auto &data = graph_.GetEdgeData(rev);
                    data.weight = new_weight;
                    data.duration = from_alias<EdgeDuration::value_type>(new_duration);
                    data.distance = new_distance;
                }
            }
        });

    // Run cell Dijkstra — either all filters or a selected subset
    TIMER_START(cell_customize);
    if (filter_indices.empty())
    {
        // All filters (default — identical to previous behavior)
        latest_metrics_ = customizeAllFilters(
            graph_, storage_, CellCustomizer{mlp_}, node_filters_);
    }
    else
    {
        // Selective: only re-run listed filters, keep previous metrics for others
        CellCustomizer customizer{mlp_};
        for (auto idx : filter_indices)
        {
            if (idx < node_filters_.size() && idx < latest_metrics_.size())
            {
                auto metric = storage_.MakeMetric();
                customizer.Customize(graph_, storage_, node_filters_[idx], metric);
                latest_metrics_[idx] = std::move(metric);
            }
        }
    }
    TIMER_STOP(cell_customize);
    double cell_time = TIMER_SEC(cell_customize);

    return cell_time;
}

} // namespace osrm::customizer
