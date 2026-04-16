#ifndef OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP
#define OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP

#include "customizer/cell_customizer.hpp"
#include "customizer/cell_metric.hpp"
#include "customizer/customizer_config.hpp"

#include "extractor/edge_based_edge.hpp"

#include "partitioner/cell_storage.hpp"
#include "partitioner/multi_level_graph.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "util/exclude_flag.hpp"
#include "util/typedefs.hpp"

#include <string>
#include <vector>

namespace osrm::customizer
{

/// Keeps the MLD graph and partition in memory between customize calls,
/// avoiding repeated disk I/O and graph construction.  The graph topology
/// is built once; subsequent calls patch edge weights in-place and re-run
/// cell Dijkstra only.
///
/// Usage:
///   InMemoryCustomizer imc;
///   imc.Initialize(config);           // one-time: loads graph, partition
///   double t = imc.Recustomize(csv);  // fast: updater + weight patch + cell Dijkstra
class InMemoryCustomizer
{
  public:
    InMemoryCustomizer() = default;

    /// One-time initialization.  Loads partition, cell storage, node data,
    /// and runs a full first customize (updater + graph build + cell Dijkstra).
    /// Builds an edge mapping for in-place weight updates on subsequent calls.
    void Initialize(const CustomizationConfig &config);

    /// Re-customize using a new speed CSV.  Runs the updater to get new
    /// edge weights, patches them in-place on the cached graph (skipping
    /// the full graph rebuild), then re-runs cell Dijkstra.
    ///
    /// Returns the wall-clock time (seconds) spent in cell Dijkstra.
    ///
    /// @param filter_indices  If non-empty, only re-run cell Dijkstra for
    ///   these filter indices (e.g. {0} = default filter only).  Filters
    ///   not listed retain their previous metrics from the last full run.
    ///   Empty (default) = all filters, identical to previous behavior.
    double Recustomize(const std::string &speed_csv_path,
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

    // Cached graph (topology fixed, weights patched in-place per Recustomize)
    partitioner::MultiLevelEdgeBasedGraph graph_;
    EdgeID num_nodes_ = 0;

    // Edge mapping for in-place updates: original_edge[i] → graph edge ID
    // Built during Initialize, used by Recustomize to skip graph rebuild.
    // fwd = source→target direction, rev = target→source direction.
    // SPECIAL_EDGEID if that direction was pruned (INVALID_EDGE_WEIGHT).
    std::vector<EdgeID> edge_to_graph_fwd_;
    std::vector<EdgeID> edge_to_graph_rev_;

    // Latest metrics from Recustomize() — one per exclude filter
    std::vector<CellMetric> latest_metrics_;
};

} // namespace osrm::customizer

#endif // OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP
