#ifndef OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP
#define OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP

#include "customizer/cell_customizer.hpp"
#include "customizer/cell_metric.hpp"
#include "customizer/customizer_config.hpp"

#include "extractor/edge_based_edge.hpp"
#include "extractor/node_data_container.hpp"
#include "extractor/packed_osm_ids.hpp"
#include "extractor/profile_properties.hpp"
#include "extractor/segment_data_container.hpp"

#include "partitioner/cell_storage.hpp"
#include "partitioner/multi_level_graph.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "util/coordinate.hpp"
#include "util/exclude_flag.hpp"
#include "util/typedefs.hpp"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace osrm::customizer
{

/// Timing breakdown returned by Recustomize().
struct RecustomizeResult
{
    double phase1_csv_s = 0;   ///< CSV parse + dirty geometry lookup
    double phase1_copy_s = 0;  ///< Segment data copy
    double phase1_update_s = 0;///< Segment weight update (parallel)
    double phase2_accum_s = 0; ///< Accumulate per-geometry weights
    double phase3_patch_s = 0; ///< Graph edge patching
    double phase4_cell_s = 0;  ///< Cell Dijkstra
    std::size_t dirty_geometries = 0;
    std::size_t edges_patched = 0;
    std::size_t newly_clean = 0;
    std::size_t dirty_cells = 0;
    double total_s = 0;        ///< Total wall time
};

/// Keeps the MLD graph, partition, and all immutable extraction data in memory
/// between customize calls.  The graph topology is built once; subsequent calls
/// parse the speed CSV, update segment weights from cached data, recompute edge
/// weights, patch the graph in-place, and re-run cell Dijkstra — with zero file
/// I/O in the hot path.
///
/// Usage:
///   InMemoryCustomizer imc;
///   imc.Initialize(config);           // one-time: loads everything
///   double t = imc.Recustomize(csv);  // fast: CSV + cached update + cell Dijkstra
class InMemoryCustomizer
{
  public:
    InMemoryCustomizer() = default;

    /// One-time initialization.  Loads partition, cell storage, extraction data,
    /// and runs a full first customize.  Builds edge mapping and caches all
    /// immutable data for subsequent Recustomize() calls.
    void Initialize(const CustomizationConfig &config);

    /// Re-customize using a new speed CSV.  Uses cached extraction data (no file
    /// I/O), patches graph weights in-place, then re-runs cell Dijkstra.
    ///
    /// Returns per-phase timing breakdown.
    ///
    /// @param filter_indices  If non-empty, only re-run cell Dijkstra for
    ///   these filter indices (e.g. {0} = default filter only).  Filters
    ///   not listed retain their previous metrics from the last full run.
    ///   Empty (default) = all filters, identical to previous behavior.
    RecustomizeResult Recustomize(const std::string &speed_csv_path,
                       const std::vector<std::size_t> &filter_indices = {});

    /// Access the metrics produced by the last Recustomize() call.
    /// One CellMetric per exclude filter (typically 4).
    const std::vector<CellMetric> &GetLatestMetrics() const { return latest_metrics_; }

    bool IsInitialized() const { return initialized_; }

  private:
    bool initialized_ = false;
    CustomizationConfig config_;

    // Cached across calls (loaded once in Initialize)
    partitioner::MultiLevelPartition mlp_;
    partitioner::CellStorage storage_;
    std::vector<std::vector<bool>> node_filters_;
    std::uint32_t connectivity_checksum_ = 0;

    // Cached extraction data (immutable between calls)
    extractor::SegmentDataContainer segment_data_base_;
    extractor::EdgeBasedNodeDataContainer node_data_;
    extractor::ProfileProperties profile_properties_;
    std::vector<util::Coordinate> coordinates_;
    extractor::PackedOSMIDs osm_node_ids_;
    std::vector<TurnPenalty> turn_weight_penalties_;
    std::vector<TurnPenalty> turn_duration_penalties_;

    // Base (freeflow) edge list from initial Updater run
    std::vector<extractor::EdgeBasedEdge> base_edge_list_;

    // Cached graph (topology fixed, weights patched in-place per Recustomize)
    partitioner::MultiLevelEdgeBasedGraph graph_;
    EdgeID num_nodes_ = 0;

    // Edge mapping for in-place updates: original_edge[i] → graph edge ID
    std::vector<EdgeID> edge_to_graph_fwd_;
    std::vector<EdgeID> edge_to_graph_rev_;

    // Reverse index: OSM segment (u,v) → list of geometry IDs containing it.
    // Enables CSV-driven Phase 1 instead of scanning all geometries.
    struct SegmentHash {
        std::size_t operator()(const std::pair<std::uint64_t, std::uint64_t> &p) const {
            // Mix both IDs — order matters (directed segments)
            return std::hash<std::uint64_t>{}(p.first) ^ (std::hash<std::uint64_t>{}(p.second) << 32);
        }
    };
    using SegmentKey = std::pair<std::uint64_t, std::uint64_t>;
    std::unordered_map<SegmentKey, std::vector<std::uint32_t>, SegmentHash> osm_to_geometries_;

    // Reverse index: geometry ID → list of base_edge_list_ indices whose source
    // node references that geometry.  Enables dirty-only Phase 3.
    std::unordered_map<std::uint32_t, std::vector<std::size_t>> geometry_to_edges_;

    // Dirty tracking: geometry IDs that were updated in the previous Recustomize
    // call.  Used to detect "newly clean" edges that need freeflow reset.
    std::unordered_set<std::uint32_t> prev_dirty_geometries_;

    // Persistent working copy of segment data. Accumulates delta updates
    // within a period; reset from segment_data_base_ for newly-clean geometries
    // at period transitions.
    extractor::SegmentDataContainer segment_data_working_;
    bool segment_data_initialized_ = false;

    // Latest metrics from Recustomize() — one per exclude filter
    std::vector<CellMetric> latest_metrics_;
};

} // namespace osrm::customizer

#endif // OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP
