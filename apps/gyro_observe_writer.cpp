#include "apps/gyro_observe_writer.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <string>
#include <string_view>
#include <system_error>

/**
 * @file gyro_observe_writer.cpp
 * @brief Q1 Observe CSV 序列化与同目录原子发布。
 */

namespace phad::apps
{
  namespace
  {

    constexpr std::string_view kGyroPacketsHeader =
        "packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,"
        "sum_dt_ns,interval_ns,status\n";
    constexpr std::string_view kGyroSamplesHeader =
        "packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,"
        "gyr_z_radps\n";

    [[nodiscard]] std::optional<SessionError> gyroObserveError(
        std::string_view code )
    {
      return SessionError{ "gyro observe:" + std::string{ code } };
    }

    template <typename WriteRows>
    [[nodiscard]] std::optional<SessionError> writeGyroCsvAtomic(
        const std::filesystem::path& path,
        std::string_view             label,
        WriteRows&&                  write_rows )
    {
      std::filesystem::path temp_path = path;
      temp_path += ".tmp";
      std::ofstream out( temp_path, std::ios::binary | std::ios::trunc );
      out.imbue( std::locale::classic() );
      if ( !out )
      {
        return gyroObserveError( std::string{ label } + "_open" );
      }
      write_rows( out );
      if ( !out )
      {
        return gyroObserveError( std::string{ label } + "_write" );
      }
      out.flush();
      if ( !out )
      {
        return gyroObserveError( std::string{ label } + "_flush" );
      }
      out.close();
      if ( out.fail() )
      {
        return gyroObserveError( std::string{ label } + "_close" );
      }

      std::error_code error;
      std::filesystem::rename( temp_path, path, error );
      if ( error )
      {
        return gyroObserveError( std::string{ label } + "_publish" );
      }
      return std::nullopt;
    }

  }  // namespace

  std::string_view gyroPacketStatusName( GyroPacketStatus status ) noexcept
  {
    switch ( status )
    {
      case GyroPacketStatus::kFirstZero:
        return "first_zero";
      case GyroPacketStatus::kGap:
        return "gap";
      case GyroPacketStatus::kEmptyNonfirst:
        return "empty_nonfirst";
      case GyroPacketStatus::kValid:
        return "valid";
    }
    return "unknown";
  }

  std::optional<SessionError> writeGyroObserveCsvs(
      const std::filesystem::path& packets_path,
      const std::filesystem::path& samples_path,
      const GyroObserveArtifacts&  artifacts )
  {
    if ( const auto error = validateGyroObserveArtifacts( artifacts ) )
    {
      return error;
    }
    if ( const auto error = writeGyroCsvAtomic(
             packets_path, "packets", [ &artifacts ]( std::ofstream& out ) {
               out << kGyroPacketsHeader;
               for ( const GyroPacketRow& row : artifacts.packets )
               {
                 out << row.packet_index << ',' << row.t_prev_ns << ','
                     << row.t_cur_ns << ',' << row.vo_segment_id << ','
                     << ( row.imu_gap ? 1 : 0 ) << ',' << row.sample_count
                     << ',' << row.sum_dt_ns << ',' << row.interval_ns << ','
                     << gyroPacketStatusName( row.status ) << '\n';
               }
             } ) )
    {
      return error;
    }
    return writeGyroCsvAtomic(
        samples_path, "samples", [ &artifacts ]( std::ofstream& out ) {
          out << std::setprecision(
              std::numeric_limits<double>::max_digits10 );
          out << kGyroSamplesHeader;
          for ( const GyroSampleRow& row : artifacts.samples )
          {
            out << row.packet_index << ',' << row.sample_index << ','
                << row.timestamp_ns << ',' << row.gyr_x_radps << ','
                << row.gyr_y_radps << ',' << row.gyr_z_radps << '\n';
          }
        } );
  }

}  // namespace phad::apps
