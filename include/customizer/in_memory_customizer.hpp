#ifndef OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP
#define OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP

#include "customizer/cell_customizer.hpp"
#include "customizer/cell_metric.hpp"
#include "customizer/customizer_config.hpp"

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
/// avoiding repeated disk I/O and graph construction.  Only the updater
/// (CSV → edge weights) and cell Dijkstra are re-run on each call.
///
/// Usage:
///   InMemoryCustomizer imc;
///   imc.Initialize(config);           // one-time: loads graph, partition
///   double t = imc.Recustomize(csv);  // fast: updater + cell Dijkstra + write metrics
///   // then reload OSRM engine to pick up new .osrm.cell_metrics
class InMemoryCustomizer
{
  public:
    InMemoryCustomizer() = default;

    /// One-time initialization.  Loads partition, cell storage, node data,
    /// and runs a full first customize (updater + graph build + cell Dijkstra).
    /// After this call the graph and partition are cached in memory.
    void Initialize(const CustomizationConfig &config);

    /// Re-customize using a new speed CSV.  Runs the updater to get new
    /// edge weights, patches the cached graph, re-runs cell Dijkstra,
    /// and writes the updated .osrm.cell_metrics file.
    ///
    /// Returns the wall-clock time (seconds) spent in cell Dijkstra.
    double Recustomize(const std::string &speed_csv_path);

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

    // Cached graph (topology fixed, weights updated per Recustomize)
    partitioner::MultiLevelEdgeBasedGraph graph_;
    EdgeID num_nodes_ = 0;

    // Latest metrics from Recustomize() — one per exclude filter
    std::vector<CellMetric> latest_metrics_;
};

} // namespace osrm::customizer

#endif // OSRM_CUSTOMIZER_IN_MEMORY_CUSTOMIZER_HPP
