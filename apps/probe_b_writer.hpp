#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <vector>

/**
 * @file probe_b_writer.hpp
 * @brief MH_05 Probe B 旁路 jsonl writer（默认关；session 在 path 非空时构造）。
 */

namespace phad::apps
{

  struct ProbeBShiftTop
  {
    std::uint64_t key  = 0;
    double        dt_m = 0.0;
  };

  /// 单行 probe_b.jsonl 记录；optional 字段表示轻/重帧差异。
  struct ProbeBFrame
  {
    std::uint64_t i      = 0;
    std::int64_t  ts_ns  = 0;
    std::optional<std::vector<std::uint64_t>> culled_ids;
    std::optional<std::uint32_t>              zombie_track_n;
    std::optional<std::uint32_t>              rejected_block_n;
    std::optional<std::uint32_t>              new_lm;
    std::optional<std::uint32_t>              shared;
    std::optional<std::uint32_t>              num_obs;
    std::optional<std::uint32_t>              lm_iterations;
    std::optional<double>                     shift_m;
    std::optional<std::vector<ProbeBShiftTop>> shift_top;
    std::optional<double>                     res_mean_px;
    std::optional<double>                     res_max_px;
    std::optional<std::uint64_t>              res_max_id;
    std::optional<bool>                       drops_skipped_this_frame;
  };

  class ProbeBWriter
  {
  public:
    explicit ProbeBWriter( const std::filesystem::path& path );
    ~ProbeBWriter();

    ProbeBWriter( const ProbeBWriter& )            = delete;
    ProbeBWriter& operator=( const ProbeBWriter& ) = delete;
    ProbeBWriter( ProbeBWriter&& )                 = delete;
    ProbeBWriter& operator=( ProbeBWriter&& )      = delete;

    void write( const ProbeBFrame& frame );

  private:
    std::ofstream m_out;
  };

}  // namespace phad::apps
