#include "phad/eval/tum_io.hpp"

#include <Eigen/Geometry>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

/**
 * @file tum_io.cpp
 * @brief TUM 轨迹格式读写实现。
 */

namespace phad::eval
{
  namespace fs = std::filesystem;

  namespace
  {

    constexpr std::int64_t kNanosPerSecond = 1'000'000'000;
    constexpr std::size_t  kFractionDigits = 9U;
    constexpr std::size_t  kFieldCount     = 8U;
    // 外部工具写出的 TUM 文件常把四元数截断到几位小数，检查只用于识别
    // 损坏的记录（全零、列错位），读入后统一归一化。
    constexpr double kUnitQuaternionTolerance = 1e-3;

    EvalError makeError( EvalErrorCode code, fs::path path,
                         std::optional<std::size_t> line, std::string field,
                         std::string cause )
    {
      return EvalError{ code, std::move( path ), line, std::move( field ),
                        std::move( cause ) };
    }

    /// 把整数纳秒写成 `<秒>.<9 位小数>`，不经过浮点。
    void writeTimestamp( std::ostream& stream, common::Timestamp timestamp )
    {
      std::int64_t seconds  = timestamp.nanoseconds() / kNanosPerSecond;
      std::int64_t fraction = timestamp.nanoseconds() % kNanosPerSecond;
      if ( fraction < 0 )
      {
        --seconds;
        fraction += kNanosPerSecond;
      }
      stream << seconds << '.' << std::setfill( '0' )
             << std::setw( static_cast<int>( kFractionDigits ) ) << fraction
             << std::setfill( ' ' );
    }

    std::vector<std::string_view> splitWhitespace( std::string_view line )
    {
      std::vector<std::string_view> fields;
      std::size_t                   begin = 0;
      while ( begin < line.size() )
      {
        const std::size_t start = line.find_first_not_of( " \t", begin );
        if ( start == std::string_view::npos )
        {
          return fields;
        }
        const std::size_t end = line.find_first_of( " \t", start );
        if ( end == std::string_view::npos )
        {
          fields.push_back( line.substr( start ) );
          return fields;
        }
        fields.push_back( line.substr( start, end - start ) );
        begin = end;
      }
      return fields;
    }

    /// 把 `<秒>[.<最多 9 位小数>]` 解析为整数纳秒。
    EvalResult<common::Timestamp> parseTimestamp( std::string_view text,
                                                  const fs::path&  path,
                                                  std::size_t      line )
    {
      const auto       dot           = text.find( '.' );
      std::string_view seconds_text  = text.substr( 0, dot );
      std::string_view fraction_text = dot == std::string_view::npos
                                           ? std::string_view{}
                                           : text.substr( dot + 1 );
      const bool       negative      = !seconds_text.empty() &&
                            seconds_text.front() == '-';

      std::int64_t seconds = 0;
      const auto   parsed  = std::from_chars(
          seconds_text.data(), seconds_text.data() + seconds_text.size(),
          seconds, 10 );
      if ( parsed.ec != std::errc{} ||
           parsed.ptr != seconds_text.data() + seconds_text.size() )
      {
        return makeError( EvalErrorCode::kInvalidRecord, path, line,
                          "timestamp",
                          "expected a decimal timestamp in seconds" );
      }
      if ( fraction_text.size() > kFractionDigits )
      {
        return makeError(
            EvalErrorCode::kInvalidRecord, path, line, "timestamp",
            "timestamp carries sub-nanosecond digits, which the nanosecond "
            "time contract cannot represent" );
      }

      std::int64_t fraction = 0;
      if ( !fraction_text.empty() )
      {
        const auto fraction_parsed = std::from_chars(
            fraction_text.data(), fraction_text.data() + fraction_text.size(),
            fraction, 10 );
        if ( fraction_parsed.ec != std::errc{} ||
             fraction_parsed.ptr !=
                 fraction_text.data() + fraction_text.size() )
        {
          return makeError( EvalErrorCode::kInvalidRecord, path, line,
                            "timestamp",
                            "expected decimal digits after the dot" );
        }
        for ( std::size_t index = fraction_text.size();
              index < kFractionDigits; ++index )
        {
          fraction *= 10;
        }
      }

      const std::int64_t magnitude =
          ( negative ? -seconds : seconds ) * kNanosPerSecond + fraction;
      return common::Timestamp{ negative ? -magnitude : magnitude };
    }

    EvalResult<double> parseFinite( std::string_view text,
                                    const fs::path& path, std::size_t line,
                                    std::string field )
    {
      double     value  = 0.0;
      const auto parsed = std::from_chars(
          text.data(), text.data() + text.size(), value );
      if ( parsed.ec != std::errc{} ||
           parsed.ptr != text.data() + text.size() )
      {
        return makeError( EvalErrorCode::kInvalidRecord, path, line,
                          std::move( field ),
                          "expected a floating-point value" );
      }
      if ( !std::isfinite( value ) )
      {
        return makeError( EvalErrorCode::kInvalidRecord, path, line,
                          std::move( field ), "value must be finite" );
      }
      return value;
    }

    EvalError mapTrajectoryError( const common::TrajectoryError&  error,
                                  const fs::path&                 path,
                                  const std::vector<std::size_t>& lines )
    {
      const std::optional<std::size_t> line =
          error.index < lines.size()
              ? std::optional<std::size_t>{ lines[ error.index ] }
              : std::nullopt;
      return makeError( EvalErrorCode::kInvalidTrajectory, path, line, {},
                        error.detail );
    }

  }  // namespace

  std::optional<EvalError> writeTum( const fs::path&           path,
                                     const common::Trajectory& trajectory )
  {
    std::ofstream stream( path, std::ios::trunc );
    if ( !stream )
    {
      return makeError( EvalErrorCode::kIoError, path, std::nullopt, {},
                        "failed to open trajectory file for writing" );
    }

    // 17 位有效数字使 double 位姿可无损往返，代价是文件略长。
    stream << std::setprecision( std::numeric_limits<double>::max_digits10 );
    for ( const common::TimedPose& pose : trajectory.poses() )
    {
      const Eigen::Vector3d    translation = pose.T_W_B.translation();
      const Eigen::Quaterniond rotation{ pose.T_W_B.linear() };
      writeTimestamp( stream, pose.timestamp );
      stream << ' ' << translation.x() << ' ' << translation.y() << ' '
             << translation.z() << ' ' << rotation.x() << ' ' << rotation.y()
             << ' ' << rotation.z() << ' ' << rotation.w() << '\n';
    }
    stream.flush();
    if ( !stream )
    {
      return makeError( EvalErrorCode::kIoError, path, std::nullopt, {},
                        "failed while writing trajectory file" );
    }
    return std::nullopt;
  }

  EvalResult<common::Trajectory> readTum( const fs::path& path )
  {
    std::ifstream stream( path );
    if ( !stream )
    {
      return makeError( EvalErrorCode::kIoError, path, std::nullopt, {},
                        "failed to open trajectory file for reading" );
    }

    std::vector<common::TimedPose> poses;
    std::vector<std::size_t>       lines;
    std::string                    line_text;
    std::size_t                    line_number = 0;
    while ( std::getline( stream, line_text ) )
    {
      ++line_number;
      if ( !line_text.empty() && line_text.back() == '\r' )
      {
        line_text.pop_back();
      }
      const auto fields = splitWhitespace( line_text );
      if ( fields.empty() || fields.front().front() == '#' )
      {
        continue;
      }
      if ( fields.size() != kFieldCount )
      {
        return makeError(
            EvalErrorCode::kInvalidRecord, path, line_number, {},
            "TUM row must contain timestamp tx ty tz qx qy qz qw" );
      }

      auto timestamp = parseTimestamp( fields[ 0 ], path, line_number );
      if ( !timestamp )
      {
        return timestamp.error();
      }

      static constexpr std::array<std::string_view, 7> kFieldNames{
          "tx", "ty", "tz", "qx", "qy", "qz", "qw" };
      std::array<double, 7> values{};
      for ( std::size_t index = 0; index < values.size(); ++index )
      {
        auto value = parseFinite( fields[ index + 1U ], path, line_number,
                                  std::string{ kFieldNames[ index ] } );
        if ( !value )
        {
          return value.error();
        }
        values[ index ] = value.value();
      }

      const Eigen::Quaterniond rotation{ values[ 6 ], values[ 3 ],
                                         values[ 4 ], values[ 5 ] };
      if ( std::abs( rotation.norm() - 1.0 ) > kUnitQuaternionTolerance )
      {
        return makeError( EvalErrorCode::kInvalidRecord, path, line_number,
                          "qx qy qz qw", "quaternion must have unit norm" );
      }

      common::TimedPose pose;
      pose.timestamp      = timestamp.value();
      pose.T_W_B.linear() = rotation.normalized().toRotationMatrix();
      pose.T_W_B.translation() =
          Eigen::Vector3d{ values[ 0 ], values[ 1 ], values[ 2 ] };
      poses.push_back( pose );
      lines.push_back( line_number );
    }
    if ( stream.bad() )
    {
      return makeError( EvalErrorCode::kIoError, path, std::nullopt, {},
                        "failed while reading trajectory file" );
    }

    auto trajectory = common::Trajectory::create( std::move( poses ) );
    if ( !trajectory )
    {
      return mapTrajectoryError( trajectory.error(), path, lines );
    }
    return std::move( trajectory ).value();
  }

}  // namespace phad::eval
