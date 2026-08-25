#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"

namespace phad::estimator::internal
{

  enum class ImuIntervalErrorCode
  {
    kEndpointOrder,
    kEndpointMismatch,
    kTooFewSamples,
    kNonIncreasingTimestamp,
    kNonFiniteMeasurement,
    kMissingBracket,
    kTimestampOverflow,
    kNonFiniteInterpolation,
    kInvalidStep,
    kClosureMismatch
  };

  struct ImuIntervalError
  {
    ImuIntervalErrorCode       m_code;
    std::optional<std::size_t> m_sample_index;
    std::string                m_detail;
  };

  struct ImuIntervalStep
  {
    Eigen::Vector3d m_acc_mean_mps2;
    Eigen::Vector3d m_gyr_mean_radps;
    std::int64_t    m_dt_ns = 0;
    double          m_dt_s  = 0.0;
  };

  struct NormalizedImuInterval
  {
    sensor::RawImuInterval              m_raw;
    std::vector<sensor::ImuMeasurement> m_nodes;
    std::vector<ImuIntervalStep>        m_steps;
    std::int64_t                        m_duration_ns = 0;
  };

  using ImuIntervalResult =
      std::variant<NormalizedImuInterval, ImuIntervalError>;

  [[nodiscard]] ImuIntervalResult normalizeRawImuInterval(
      const sensor::RawImuInterval&    raw,
      std::optional<common::Timestamp> expected_t_begin,
      common::Timestamp                expected_t_end );

}  // namespace phad::estimator::internal
