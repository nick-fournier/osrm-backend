#ifndef OSRM_CUSTOMIZER_FILES_HPP
#define OSRM_CUSTOMIZER_FILES_HPP

#include "customizer/serialization.hpp"

#include "storage/tar.hpp"

#include "util/integer_range.hpp"
#include "util/log.hpp"

#include <unordered_map>

namespace osrm::customizer::files
{

// reads .osrm.cell_metrics file (multi-period aware, backward-compatible)
template <typename CellMetricT>
inline void readCellMetrics(const std::filesystem::path &path,
                            std::unordered_map<std::string, std::vector<CellMetricT>> &metrics)
{
    static_assert(std::is_same<CellMetricView, CellMetricT>::value ||
                      std::is_same<CellMetric, CellMetricT>::value,
                  "");

    const auto fingerprint = storage::tar::FileReader::VerifyFingerprint;
    storage::tar::FileReader reader{path, fingerprint};

    for (auto &pair : metrics)
    {
        const auto &metric_name = pair.first;
        auto &metric_exclude_classes = pair.second;

        auto prefix = "/mld/metrics/" + metric_name + "/exclude";
        auto num_exclude_classes = reader.ReadElementCount64(prefix);
        metric_exclude_classes.resize(num_exclude_classes);

        auto id = 0;
        for (auto &metric : metric_exclude_classes)
        {
            serialization::read(reader, prefix + "/" + std::to_string(id++), metric);
        }
    }
}

// reads multi-period cell metrics from .osrm.cell_metrics
// Returns map: metric_name -> vector of periods, each period is vector of exclude-class metrics
template <typename CellMetricT>
inline void readPeriodCellMetrics(
    const std::filesystem::path &path,
    std::unordered_map<std::string, std::vector<std::vector<CellMetricT>>> &period_metrics,
    std::size_t num_periods)
{
    static_assert(std::is_same<CellMetricView, CellMetricT>::value ||
                      std::is_same<CellMetric, CellMetricT>::value,
                  "");

    const auto fingerprint = storage::tar::FileReader::VerifyFingerprint;
    storage::tar::FileReader reader{path, fingerprint};

    for (auto &pair : period_metrics)
    {
        const auto &metric_name = pair.first;
        auto &periods = pair.second;
        periods.resize(num_periods);

        for (std::size_t period = 0; period < num_periods; ++period)
        {
            auto period_prefix =
                "/mld/metrics/" + metric_name + "/period/" + std::to_string(period) + "/exclude";

            if (!reader.HasEntry(period_prefix + ".meta"))
                continue; // sparse: period not stored, will fall back to base (period 0)

            auto num_exclude_classes = reader.ReadElementCount64(period_prefix);
            periods[period].resize(num_exclude_classes);

            for (std::size_t e = 0; e < num_exclude_classes; ++e)
            {
                serialization::read(
                    reader, period_prefix + "/" + std::to_string(e), periods[period][e]);
            }
        }
    }
}

// writes .osrm.cell_metrics file (single-period, backward-compatible)
template <typename CellMetricT>
inline void
writeCellMetrics(const std::filesystem::path &path,
                 const std::unordered_map<std::string, std::vector<CellMetricT>> &metrics)
{
    static_assert(std::is_same<CellMetricView, CellMetricT>::value ||
                      std::is_same<CellMetric, CellMetricT>::value,
                  "");

    const auto fingerprint = storage::tar::FileWriter::GenerateFingerprint;
    storage::tar::FileWriter writer{path, fingerprint};

    for (const auto &pair : metrics)
    {
        const auto &metric_name = pair.first;
        const auto &metric_exclude_classes = pair.second;

        auto prefix = "/mld/metrics/" + metric_name + "/exclude";
        writer.WriteElementCount64(prefix, metric_exclude_classes.size());

        auto id = 0;
        for (auto &exclude_metric : metric_exclude_classes)
        {
            serialization::write(writer, prefix + "/" + std::to_string(id++), exclude_metric);
        }
    }
}

// writes multi-period cell metrics to .osrm.cell_metrics
// period_metrics: vector of (period_index, exclude_metrics) pairs — sparse, only populated periods
// Also writes legacy (non-period) paths from period 0 for backward compatibility
template <typename CellMetricT>
inline void writeMultiPeriodCellMetrics(
    const std::filesystem::path &path,
    const std::string &metric_name,
    const std::vector<std::pair<std::size_t, std::vector<CellMetricT>>> &period_metrics)
{
    static_assert(std::is_same<CellMetricView, CellMetricT>::value ||
                      std::is_same<CellMetric, CellMetricT>::value,
                  "");

    const auto fingerprint = storage::tar::FileWriter::GenerateFingerprint;
    storage::tar::FileWriter writer{path, fingerprint};

    // Write period count metadata
    writer.WriteElementCount64("/mld/period_count", period_metrics.size());

    for (const auto &[period_index, exclude_metrics] : period_metrics)
    {
        // Write period-indexed metrics
        auto prefix = "/mld/metrics/" + metric_name + "/period/" +
                      std::to_string(period_index) + "/exclude";
        writer.WriteElementCount64(prefix, exclude_metrics.size());

        std::size_t id = 0;
        for (const auto &exclude_metric : exclude_metrics)
        {
            serialization::write(writer, prefix + "/" + std::to_string(id++), exclude_metric);
        }

        // Period 0 also written under legacy paths for backward compatibility
        if (period_index == 0)
        {
            auto legacy_prefix = "/mld/metrics/" + metric_name + "/exclude";
            writer.WriteElementCount64(legacy_prefix, exclude_metrics.size());

            id = 0;
            for (const auto &exclude_metric : exclude_metrics)
            {
                serialization::write(
                    writer, legacy_prefix + "/" + std::to_string(id++), exclude_metric);
            }
        }
    }
}

// reads .osrm.mldgr file
template <typename MultiLevelGraphT>
inline void readGraph(const std::filesystem::path &path,
                      MultiLevelGraphT &graph,
                      std::uint32_t &connectivity_checksum)
{
    static_assert(std::is_same<customizer::MultiLevelEdgeBasedGraphView, MultiLevelGraphT>::value ||
                      std::is_same<customizer::MultiLevelEdgeBasedGraph, MultiLevelGraphT>::value,
                  "");

    storage::tar::FileReader reader{path, storage::tar::FileReader::VerifyFingerprint};

    reader.ReadInto("/mld/connectivity_checksum", connectivity_checksum);
    serialization::read(reader, "/mld/multilevelgraph", graph);
}

// writes .osrm.mldgr file
template <typename MultiLevelGraphT>
inline void writeGraph(const std::filesystem::path &path,
                       const MultiLevelGraphT &graph,
                       const std::uint32_t connectivity_checksum)
{
    static_assert(std::is_same<customizer::MultiLevelEdgeBasedGraphView, MultiLevelGraphT>::value ||
                      std::is_same<customizer::MultiLevelEdgeBasedGraph, MultiLevelGraphT>::value,
                  "");

    storage::tar::FileWriter writer{path, storage::tar::FileWriter::GenerateFingerprint};

    writer.WriteElementCount64("/mld/connectivity_checksum", 1);
    writer.WriteFrom("/mld/connectivity_checksum", connectivity_checksum);
    serialization::write(writer, "/mld/multilevelgraph", graph);
}
} // namespace osrm::customizer::files

#endif
