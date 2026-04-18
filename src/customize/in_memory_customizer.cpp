#include "customizer/in_memory_customizer.hpp"

#include "customizer/cell_customizer.hpp"
#include "customizer/files.hpp"

#include "extractor/files.hpp"

#include "partitioner/cell_storage.hpp"
#include "partitioner/edge_based_graph_reader.hpp"
#include "partitioner/files.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "updater/csv_source.hpp"
#include "updater/updater.hpp"

#include "util/coordinate_calculation.hpp"
#include "util/exclude_flag.hpp"
#include "util/for_each_pair.hpp"
#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/timing_util.hpp"

#include <boost/numeric/conversion/cast.hpp>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

#include <numeric>
#include <ranges>

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

        if (source == target) { it = end_interval; continue; }

        auto first_bwd = std::find_if(it, end_interval,
            [](const auto &e) { return !e.data.forward && e.data.backward; });

        if (it != first_bwd)
            output.push_back(OutputEdgeT{source, target, it->data});
        if (first_bwd != end_interval)
            output.push_back(OutputEdgeT{source, target, first_bwd->data});

        it = end_interval;
    }

    return output;
}

// Replicated from updater.cpp — converts speed+distance to OSRM duration
inline SegmentDuration convertToDuration(double speed_in_kmh, double distance_in_meters)
{
    if (speed_in_kmh <= 0.)
        return INVALID_SEGMENT_DURATION;
    const auto speed_in_ms = speed_in_kmh / 3.6;
    const auto duration = distance_in_meters / speed_in_ms;
    auto segment_duration = std::max<SegmentDuration>(
        {1}, {boost::numeric_cast<SegmentDuration::value_type>(std::round(duration * 10.))});
    if (segment_duration >= INVALID_SEGMENT_DURATION)
        segment_duration = MAX_SEGMENT_DURATION;
    return segment_duration;
}

// Replicated from updater.cpp — handles optional rate column
inline SegmentWeight convertToWeight(const extractor::ProfileProperties &properties,
                                     const SegmentWeight &existing_weight,
                                     const updater::SpeedSource &value,
                                     double distance_in_meters)
{
    double rate;
    if (!value.rate)
    {
        rate = value.speed / 3.6;
    }
    else
    {
        rate = *value.rate;
        if (!std::isfinite(rate))
            return existing_weight;
    }

    if (rate <= 0.)
        return INVALID_SEGMENT_WEIGHT;

    const auto weight_multiplier = properties.GetWeightMultiplier();
    const auto weight = distance_in_meters / rate;
    auto segment_weight = std::max<SegmentWeight>(
        {1}, {boost::numeric_cast<SegmentWeight::value_type>(
                  std::round(weight * weight_multiplier))});
    if (segment_weight >= INVALID_SEGMENT_WEIGHT)
        segment_weight = MAX_SEGMENT_WEIGHT;
    return segment_weight;
}
} // namespace

void InMemoryCustomizer::Initialize(const CustomizationConfig &config)
{
    config_ = config;

    TIMER_START(init);

    // Load partition and cell storage
    partitioner::files::readPartition(config.GetPath(".osrm.partition"), mlp_);
    partitioner::files::readCells(config.GetPath(".osrm.cells"), storage_);

    // Load and cache all immutable extraction data
    const auto &ucfg = config.updater_config;
    extractor::files::readNodeData(config.GetPath(".osrm.ebg_nodes"), node_data_);
    extractor::files::readProfileProperties(config.GetPath(".osrm.properties"),
                                            profile_properties_);
    extractor::files::readSegmentData(ucfg.GetPath(".osrm.geometry"), segment_data_base_);
    extractor::files::readNodes(ucfg.GetPath(".osrm.nbg_nodes"), coordinates_, osm_node_ids_);
    extractor::files::readTurnWeightPenalty(
        ucfg.GetPath(".osrm.turn_weight_penalties"), turn_weight_penalties_);
    extractor::files::readTurnDurationPenalty(
        ucfg.GetPath(".osrm.turn_duration_penalties"), turn_duration_penalties_);

    // Run the updater once to get the initial (freeflow) edge list
    auto init_updater_config = config.updater_config;
    init_updater_config.skip_geometry_write = true;
    updater::Updater updater(init_updater_config);
    std::vector<EdgeWeight> node_weights;
    std::vector<EdgeDuration> node_durations;
    num_nodes_ = updater.LoadAndUpdateEdgeExpandedGraph(
        base_edge_list_, node_weights, node_durations, connectivity_checksum_);

    for (auto &w : node_weights)
        w &= EdgeWeight{0x7fffffff};

    // Build node filters
    node_filters_ = util::excludeFlagsToNodeFilter(
        node_weights.size(), node_data_, profile_properties_);

    // Build the MLD graph using no-merge split for stable topology
    auto directed = partitioner::splitBidirectionalEdges(base_edge_list_);
    auto tidied = prepareEdgesNoMerge<
        typename partitioner::MultiLevelEdgeBasedGraph::InputEdge>(std::move(directed));
    graph_ = partitioner::MultiLevelEdgeBasedGraph(mlp_, num_nodes_, tidied);

    // Build edge mapping: turn_id -> original index, then scan graph
    {
        std::unordered_map<NodeID, std::size_t> turn_to_orig;
        turn_to_orig.reserve(base_edge_list_.size());
        for (std::size_t i = 0; i < base_edge_list_.size(); ++i)
            turn_to_orig[base_edge_list_[i].data.turn_id] = i;

        edge_to_graph_fwd_.assign(base_edge_list_.size(), SPECIAL_EDGEID);
        edge_to_graph_rev_.assign(base_edge_list_.size(), SPECIAL_EDGEID);

        for (NodeID n = 0; n < graph_.GetNumberOfNodes(); ++n)
        {
            for (auto e : graph_.GetAdjacentEdgeRange(n))
            {
                const auto &data = graph_.GetEdgeData(e);
                auto it = turn_to_orig.find(data.turn_id);
                if (it == turn_to_orig.end())
                    continue;
                auto orig_idx = it->second;
                const auto &orig = base_edge_list_[orig_idx];

                if (n == orig.source)
                    edge_to_graph_fwd_[orig_idx] = e;
                else if (n == orig.target)
                    edge_to_graph_rev_[orig_idx] = e;
            }
        }
    }

    // Build reverse index: OSM segment (u,v) → geometry IDs
    {
        using DirectionalGeometryID = extractor::SegmentDataContainer::DirectionalGeometryID;
        auto n_geometries = segment_data_base_.GetNumberOfGeometries();
        for (DirectionalGeometryID gid = 0; gid < n_geometries; ++gid)
        {
            auto nodes_range = segment_data_base_.GetForwardGeometry(gid);
            for (std::size_t i = 0; i + 1 < nodes_range.size(); ++i)
            {
                auto osm_u = osm_node_ids_[nodes_range[i]];
                auto osm_v = osm_node_ids_[nodes_range[i + 1]];
                if (osm_u == osm_v)
                    continue;
                // Use Segment to extract uint64 from packed OSMNodeID proxy
                updater::Segment seg(osm_u, osm_v);
                osm_to_geometries_[SegmentKey(seg.from, seg.to)].push_back(gid);
                osm_to_geometries_[SegmentKey(seg.to, seg.from)].push_back(gid);
            }
        }
        util::Log() << "Built OSM→geometry index: " << osm_to_geometries_.size()
                    << " segment keys";
    }

    // Build reverse index: geometry ID → base edge indices
    {
        for (std::size_t i = 0; i < base_edge_list_.size(); ++i)
        {
            auto geometry_id = node_data_.GetGeometryID(base_edge_list_[i].source);
            geometry_to_edges_[geometry_id.id].push_back(i);
        }
        util::Log() << "Built geometry→edge index: " << geometry_to_edges_.size()
                    << " geometries → " << base_edge_list_.size() << " edges";
    }

    // Run initial cell customization (all filters)
    latest_metrics_ = customizeAllFilters(graph_, storage_, CellCustomizer{mlp_}, node_filters_);

    TIMER_STOP(init);
    util::Log() << "InMemoryCustomizer initialized in " << TIMER_SEC(init) << "s"
                << " (" << graph_.GetNumberOfEdges() << " edges, "
                << graph_.GetNumberOfNodes() << " nodes)";

    initialized_ = true;
}

RecustomizeResult InMemoryCustomizer::Recustomize(const std::string &speed_csv_path,
                                       const std::vector<std::size_t> &filter_indices)
{
    BOOST_ASSERT_MSG(initialized_, "Must call Initialize() before Recustomize()");
    TIMER_START(total);
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism,
                           config_.requested_num_threads > 0
                               ? config_.requested_num_threads
                               : tbb::this_task_arena::max_concurrency());

    // --- Phase 1: Parse CSV and update only affected geometries ---
    TIMER_START(phase1);

    auto segment_speed_lookup =
        updater::csv::readSegmentValues({speed_csv_path});

    // Collect unique geometry IDs that contain at least one CSV-matched segment
    std::unordered_set<std::uint32_t> dirty_geometry_ids;
    for (const auto &entry : segment_speed_lookup.lookup)
    {
        const auto &seg = entry.first;
        auto it = osm_to_geometries_.find(SegmentKey(seg.from, seg.to));
        if (it != osm_to_geometries_.end())
        {
            for (auto gid : it->second)
                dirty_geometry_ids.insert(gid);
        }
    }

    // Use persistent working copy instead of copying base every call.
    // First call: initialize from base. Subsequent calls: apply delta on top.
    TIMER_STOP(phase1);
    TIMER_START(phase1b);
    if (!segment_data_initialized_)
    {
        segment_data_working_ = segment_data_base_;
        segment_data_initialized_ = true;
    }
    // Reset newly-clean geometries to freeflow from base
    std::unordered_set<std::uint32_t> newly_clean_gids;
    for (auto gid : prev_dirty_geometries_)
    {
        if (dirty_geometry_ids.find(gid) == dirty_geometry_ids.end())
            newly_clean_gids.insert(gid);
    }
    for (auto gid : newly_clean_gids)
    {
        // Copy fwd/rev weights and durations from base for this geometry.
        // Must materialize through value type due to PackedVector proxy.
        auto base_fwd_w = segment_data_base_.GetForwardWeights(gid);
        auto base_fwd_d = segment_data_base_.GetForwardDurations(gid);
        auto work_fwd_w = segment_data_working_.GetForwardWeights(gid);
        auto work_fwd_d = segment_data_working_.GetForwardDurations(gid);
        for (std::size_t i = 0; i < base_fwd_w.size(); ++i)
        {
            const SegmentWeight w = base_fwd_w[i];
            const SegmentDuration d = base_fwd_d[i];
            work_fwd_w[i] = w;
            work_fwd_d[i] = d;
        }
        auto base_rev_w = segment_data_base_.GetReverseWeights(gid);
        auto base_rev_d = segment_data_base_.GetReverseDurations(gid);
        auto work_rev_w = segment_data_working_.GetReverseWeights(gid);
        auto work_rev_d = segment_data_working_.GetReverseDurations(gid);
        for (std::size_t i = 0; i < base_rev_w.size(); ++i)
        {
            const SegmentWeight w = base_rev_w[i];
            const SegmentDuration d = base_rev_d[i];
            work_rev_w[i] = w;
            work_rev_d[i] = d;
        }
    }
    auto &segment_data = segment_data_working_;
    TIMER_STOP(phase1b);

    TIMER_START(phase1c);
    tbb::concurrent_vector<GeometryID> updated_segments;
    using DirectionalGeometryID = extractor::SegmentDataContainer::DirectionalGeometryID;

    // Convert dirty set to sorted vector for parallel iteration
    std::vector<DirectionalGeometryID> dirty_gids(dirty_geometry_ids.begin(),
                                                   dirty_geometry_ids.end());
    tbb::parallel_sort(dirty_gids.begin(), dirty_gids.end());

    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, dirty_gids.size()),
        [&](const auto &range)
    {
        std::vector<double> segment_lengths;
        for (auto idx = range.begin(); idx < range.end(); ++idx)
        {
            auto geometry_id = dirty_gids[idx];
            auto nodes_range = segment_data.GetForwardGeometry(geometry_id);
            segment_lengths.clear();
            segment_lengths.reserve(nodes_range.size() + 1);
            util::for_each_pair(nodes_range, [&](const auto &u, const auto &v) {
                segment_lengths.push_back(
                    util::coordinate_calculation::greatCircleDistance(
                        coordinates_[u], coordinates_[v]));
            });

            // Forward direction
            auto fwd_weights = segment_data.GetForwardWeights(geometry_id);
            auto fwd_durations = segment_data.GetForwardDurations(geometry_id);
            bool fwd_updated = false;
            for (std::size_t i = 0; i < fwd_weights.size(); ++i)
            {
                auto u = osm_node_ids_[nodes_range[i]];
                auto v = osm_node_ids_[nodes_range[i + 1]];
                if (u == v) continue;
                if (auto value = segment_speed_lookup({u, v}))
                {
                    fwd_weights[i] = convertToWeight(
                        profile_properties_, fwd_weights[i], *value, segment_lengths[i]);
                    fwd_durations[i] = convertToDuration(value->speed, segment_lengths[i]);
                    fwd_updated = true;
                }
            }
            if (fwd_updated)
                updated_segments.push_back(GeometryID{geometry_id, true});

            // Reverse direction
            auto rev_weights =
                segment_data.GetReverseWeights(geometry_id) | std::views::reverse;
            auto rev_durations =
                segment_data.GetReverseDurations(geometry_id) | std::views::reverse;
            bool rev_updated = false;
            for (std::size_t i = 0; i < rev_weights.size(); ++i)
            {
                auto u = osm_node_ids_[nodes_range[i]];
                auto v = osm_node_ids_[nodes_range[i + 1]];
                if (u == v) continue;
                if (auto value = segment_speed_lookup({v, u}))
                {
                    rev_weights[i] = convertToWeight(
                        profile_properties_, rev_weights[i], *value, segment_lengths[i]);
                    rev_durations[i] = convertToDuration(value->speed, segment_lengths[i]);
                    rev_updated = true;
                }
            }
            if (rev_updated)
                updated_segments.push_back(GeometryID{geometry_id, false});
        }
    });
    TIMER_STOP(phase1c);

    // --- Phase 2: Accumulate segment weights per geometry (unchanged — already dirty-only) ---
    TIMER_START(phase2);

    tbb::parallel_sort(updated_segments.begin(), updated_segments.end(),
        [](const GeometryID lhs, const GeometryID rhs)
        { return std::tie(lhs.id, lhs.forward) < std::tie(rhs.id, rhs.forward); });

    using WeightAndDuration = std::tuple<EdgeWeight, EdgeDuration>;
    std::vector<WeightAndDuration> accumulated(updated_segments.size());
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, updated_segments.size()),
        [&](const auto &range) {
            for (auto idx = range.begin(); idx < range.end(); ++idx)
            {
                auto gid = updated_segments[idx];
                EdgeWeight w{0};
                EdgeDuration d{0};
                if (gid.forward)
                {
                    const auto weights = segment_data.GetForwardWeights(gid.id);
                    for (const SegmentWeight sw : weights)
                    {
                        if (sw == INVALID_SEGMENT_WEIGHT) { w = INVALID_EDGE_WEIGHT; break; }
                        w += alias_cast<EdgeWeight>(sw);
                    }
                    const auto durations = segment_data.GetForwardDurations(gid.id);
                    d = alias_cast<EdgeDuration>(
                        std::accumulate(durations.begin(), durations.end(), SegmentDuration{0}));
                }
                else
                {
                    const auto weights = segment_data.GetReverseWeights(gid.id);
                    for (const SegmentWeight sw : weights)
                    {
                        if (sw == INVALID_SEGMENT_WEIGHT) { w = INVALID_EDGE_WEIGHT; break; }
                        w += alias_cast<EdgeWeight>(SegmentWeight(sw));
                    }
                    const auto durations = segment_data.GetReverseDurations(gid.id);
                    d = alias_cast<EdgeDuration>(
                        std::accumulate(durations.begin(), durations.end(), SegmentDuration{0}));
                }
                accumulated[idx] = {w, d};
            }
        });
    TIMER_STOP(phase2);

    // --- Phase 3: Patch graph in-place — dirty edges + newly-clean edges only ---
    TIMER_START(phase3);

    // newly_clean_gids already computed in Phase 1b (segment_data reset)

    // Collect all base edge indices that need updating
    std::vector<std::size_t> edges_to_patch;
    edges_to_patch.reserve(dirty_geometry_ids.size() * 2 + newly_clean_gids.size() * 2);
    for (auto gid : dirty_geometry_ids)
    {
        auto it = geometry_to_edges_.find(gid);
        if (it != geometry_to_edges_.end())
            edges_to_patch.insert(edges_to_patch.end(), it->second.begin(), it->second.end());
    }
    for (auto gid : newly_clean_gids)
    {
        auto it = geometry_to_edges_.find(gid);
        if (it != geometry_to_edges_.end())
            edges_to_patch.insert(edges_to_patch.end(), it->second.begin(), it->second.end());
    }
    // Deduplicate (a geometry_id could appear in both fwd and rev)
    tbb::parallel_sort(edges_to_patch.begin(), edges_to_patch.end());
    edges_to_patch.erase(std::unique(edges_to_patch.begin(), edges_to_patch.end()),
                         edges_to_patch.end());

    auto turn_wp = turn_weight_penalties_;
    auto turn_dp = turn_duration_penalties_;

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, edges_to_patch.size()),
        [&](const auto &range)
        {
            for (auto idx = range.begin(); idx < range.end(); ++idx)
            {
                auto i = edges_to_patch[idx];
                const auto &base_edge = base_edge_list_[i];
                auto geometry_id = node_data_.GetGeometryID(base_edge.source);

                // Check if this edge's geometry was updated by the CSV
                auto it = std::lower_bound(
                    updated_segments.begin(), updated_segments.end(), geometry_id,
                    [](const GeometryID l, const GeometryID r)
                    { return std::tie(l.id, l.forward) < std::tie(r.id, r.forward); });

                EdgeWeight new_weight;
                EdgeDuration new_duration;

                if (it != updated_segments.end() && it->id == geometry_id.id &&
                    it->forward == geometry_id.forward)
                {
                    std::tie(new_weight, new_duration) =
                        accumulated[it - updated_segments.begin()];

                    if (new_weight == INVALID_EDGE_WEIGHT)
                    {
                        auto patch = [&](EdgeID eid) {
                            if (eid != SPECIAL_EDGEID)
                                graph_.GetEdgeData(eid).weight = INVALID_EDGE_WEIGHT;
                        };
                        patch(edge_to_graph_fwd_[i]);
                        patch(edge_to_graph_rev_[i]);
                        continue;
                    }

                    // Apply turn penalties with min-weight clamping (matches updater.cpp)
                    auto twp = turn_wp[base_edge.data.turn_id];
                    auto tdp = turn_dp[base_edge.data.turn_id];
                    const auto num_nodes =
                        segment_data.GetForwardGeometry(geometry_id.id).size();
                    const auto w_min = to_alias<EdgeWeight>(num_nodes);
                    if (alias_cast<EdgeWeight>(twp) + new_weight < w_min)
                    {
                        if (twp < TurnPenalty{0})
                            twp = alias_cast<TurnPenalty>(w_min - new_weight);
                        else
                            new_weight = w_min;
                    }
                    const auto d_min = to_alias<EdgeDuration>(num_nodes);
                    if (alias_cast<EdgeDuration>(tdp) + new_duration < d_min)
                    {
                        if (tdp < TurnPenalty{0})
                            tdp = alias_cast<TurnPenalty>(d_min - new_duration);
                        else
                            new_duration = d_min;
                    }

                    new_weight = new_weight + alias_cast<EdgeWeight>(twp);
                    new_duration = new_duration + alias_cast<EdgeDuration>(tdp);
                }
                else
                {
                    // Geometry not in CSV (newly clean) — reset to freeflow
                    new_weight = std::max(base_edge.data.weight, EdgeWeight{1});
                    new_duration = to_alias<EdgeDuration>(base_edge.data.duration);
                }

                const auto new_distance = base_edge.data.distance;

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

    // Update dirty tracking for next call
    prev_dirty_geometries_ = std::move(dirty_geometry_ids);
    TIMER_STOP(phase3);

    // --- Phase 3b: Collect dirty nodes → dirty cells per level ---
    TIMER_START(phase3b);
    std::unordered_set<NodeID> dirty_nodes;
    dirty_nodes.reserve(edges_to_patch.size() * 2);
    for (auto i : edges_to_patch)
    {
        auto src = base_edge_list_[i].source;
        dirty_nodes.insert(src);
        // Also collect target nodes from graph edges
        if (auto fwd = edge_to_graph_fwd_[i]; fwd != SPECIAL_EDGEID)
            dirty_nodes.insert(graph_.GetTarget(fwd));
        if (auto rev = edge_to_graph_rev_[i]; rev != SPECIAL_EDGEID)
            dirty_nodes.insert(graph_.GetTarget(rev));
    }

    auto num_levels = mlp_.GetNumberOfLevels();
    std::vector<std::unordered_set<CellID>> dirty_cell_sets(num_levels);
    for (auto node : dirty_nodes)
    {
        for (std::size_t level = 1; level < num_levels; ++level)
            dirty_cell_sets[level].insert(mlp_.GetCell(level, node));
    }

    // Convert to sorted vectors for parallel_for
    std::vector<std::vector<CellID>> dirty_cells_per_level(num_levels);
    std::size_t total_dirty_cells = 0;
    for (std::size_t level = 1; level < num_levels; ++level)
    {
        dirty_cells_per_level[level].assign(
            dirty_cell_sets[level].begin(), dirty_cell_sets[level].end());
        std::sort(dirty_cells_per_level[level].begin(), dirty_cells_per_level[level].end());
        total_dirty_cells += dirty_cells_per_level[level].size();
    }
    TIMER_STOP(phase3b);

    // --- Phase 4: Cell Dijkstra (dirty cells only) ---
    TIMER_START(cell_customize);
    if (filter_indices.empty())
    {
        CellCustomizer customizer{mlp_};
        for (std::size_t idx = 0; idx < node_filters_.size(); ++idx)
        {
            customizer.Customize(graph_, storage_, node_filters_[idx],
                                 latest_metrics_[idx], dirty_cells_per_level);
        }
    }
    else
    {
        CellCustomizer customizer{mlp_};
        for (auto idx : filter_indices)
        {
            if (idx < node_filters_.size() && idx < latest_metrics_.size())
            {
                customizer.Customize(graph_, storage_, node_filters_[idx],
                                     latest_metrics_[idx], dirty_cells_per_level);
            }
        }
    }
    TIMER_STOP(cell_customize);
    TIMER_STOP(total);

    RecustomizeResult result;
    result.phase1_csv_s = TIMER_SEC(phase1);
    result.phase1_copy_s = TIMER_SEC(phase1b);
    result.phase1_update_s = TIMER_SEC(phase1c);
    result.phase2_accum_s = TIMER_SEC(phase2);
    result.phase3_patch_s = TIMER_SEC(phase3) + TIMER_SEC(phase3b);
    result.phase4_cell_s = TIMER_SEC(cell_customize);
    result.dirty_geometries = dirty_gids.size();
    result.edges_patched = edges_to_patch.size();
    result.newly_clean = newly_clean_gids.size();
    result.dirty_cells = total_dirty_cells;
    result.total_s = TIMER_SEC(total);
    return result;
}

} // namespace osrm::customizer
