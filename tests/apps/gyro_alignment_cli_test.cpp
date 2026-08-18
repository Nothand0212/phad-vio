#include <gtest/gtest.h>
#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

#include "apps/gyro_alignment.hpp"
#include "apps/gyro_alignment_runner.hpp"
#include "phad/eval/tum_io.hpp"

#ifdef __linux__
#include <fcntl.h>
#include <sys/ptrace.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/user.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#endif

namespace
{

  using phad::apps::GyroAlignmentAnalyzerIdentity;
  using phad::apps::GyroAlignmentErrorCode;
  using phad::apps::GyroAlignmentFit;
  using phad::apps::GyroAlignmentFitStatus;
  using phad::apps::GyroAlignmentInputIdentity;
  using phad::apps::GyroAlignmentProtocolDescriptor;
  using phad::apps::GyroAlignmentProtocolIdentity;
  using phad::apps::GyroAlignmentProvenance;
  using phad::apps::GyroAlignmentResult;
  using phad::apps::GyroAlignmentStatus;
  using phad::apps::makeGyroAlignmentV1ProtocolDescriptor;
  using phad::apps::runGyroAlignment;

  constexpr std::string_view kSyntheticProtocolId =
      "PHAD-M4-Q3-GYRO-ALIGN-SYNTHETIC-TEST-V1";
  constexpr std::array<std::string_view, 4> kInputNames{
      "est.tum", "diag.csv", "gyro_packets.csv", "gyro_samples.csv" };
  constexpr std::array<std::string_view, 10> kFitArtifactKeys{
      "status", "bias_radps", "singular_values",
      "rank", "condition", "eligible_duration_ns",
      "packet_count", "loss_zero_rad2", "loss_fit_rad2",
      "nonlinear_nonincrease" };
  constexpr std::string_view kDiagHeader =
      "timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,"
      "window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,"
      "num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,"
      "pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px,"
      "is_keyframe,num_disparity\n";
  constexpr std::string_view kPacketHeader =
      "packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,"
      "sum_dt_ns,interval_ns,status\n";
  constexpr std::string_view kSampleHeader =
      "packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,"
      "gyr_z_radps\n";
  constexpr std::string_view kBlocksHeader =
      "schema_version,block_index,t_start_ns,t_end_ns,segment_id,packet_count,"
      "r_zero_x_rad,r_zero_y_rad,r_zero_z_rad,r_fit_x_rad,r_fit_y_rad,"
      "r_fit_z_rad,loss_zero_rad2,loss_fit_rad2,improved\n";

  struct ScopedDirectory
  {
    explicit ScopedDirectory( std::string_view label )
    {
      static std::atomic_uint64_t serial{ 0U };
      std::ostringstream          name;
      name << "phad_q3_cli_" << label << '_' << serial.fetch_add( 1U );
#ifdef __linux__
      name << '_' << static_cast<unsigned long>( ::getpid() );
#endif
      path = std::filesystem::temp_directory_path() / name.str();
      std::filesystem::create_directories( path );
    }

    ~ScopedDirectory() { std::filesystem::remove_all( path ); }

    ScopedDirectory( const ScopedDirectory& )            = delete;
    ScopedDirectory& operator=( const ScopedDirectory& ) = delete;

    std::filesystem::path path;
  };

  struct FixtureBytes
  {
    std::array<std::string, 4>      files;
    std::array<std::string_view, 4> hashes;
  };

  constexpr std::array<std::string_view, 4> kFirstZeroHashes{
      "12d730a2c28778c6239e858e8faa569c06e0f42d399e39c1c5646c874ca6fc3f",
      "083fac02eeee21b787e02518e43e8b409859317443cbef0abb964dd85a7921ae",
      "749d6e057abeedf4ca4369729b54e9bbd5a4364ad0d2aec6eb54a8c1d6e70133",
      "a89b57f602488d7d6b048d7ac2fe9c22a5278d2cb7bfbeaa8da5fde63423ae89",
  };
  constexpr std::array<std::string_view, 4> kZeroStationaryHashes{
      "3ac54a4a9f2b9e0a6a450b03137bc3c9b8f700b572ec612338033ed37ac785c3",
      "59402956f862c9a0dbc68a5e798126f0a639e17efe1431233d2cb712d5b103db",
      "924fca9311f1d78a419b1438d0761785393a8308f3de1e18751244c0eb6ca6b0",
      "23c3898895be0e0a10b0a36ecd8ddac1a3e65fc434e0a188e8293accd759d937",
  };
  constexpr std::array<std::string_view, 4> kBiasStationaryHashes{
      "3ac54a4a9f2b9e0a6a450b03137bc3c9b8f700b572ec612338033ed37ac785c3",
      "59402956f862c9a0dbc68a5e798126f0a639e17efe1431233d2cb712d5b103db",
      "924fca9311f1d78a419b1438d0761785393a8308f3de1e18751244c0eb6ca6b0",
      "c454ab49868943dece9f2605760e98a6a41ccc87c5007f2710665cfaf85347be",
  };
  constexpr std::array<std::string_view, 4> kMixedBiasStationaryHashes{
      "3ac54a4a9f2b9e0a6a450b03137bc3c9b8f700b572ec612338033ed37ac785c3",
      "59402956f862c9a0dbc68a5e798126f0a639e17efe1431233d2cb712d5b103db",
      "924fca9311f1d78a419b1438d0761785393a8308f3de1e18751244c0eb6ca6b0",
      "8b33530c977affc61dcfe488bc7829e69c48fab32c93a13a279a2371ea24eed6",
  };
  constexpr std::array<std::string_view, 4> kMarginStationaryHashes{
      "b339214733c16d78360be6eb48422ee6384d1868ddfa28f959b164e5a2fb752e",
      "c7838fa1d5397c11ea85f4775fd5da2c4fe512328da874636b3086746f744b92",
      "fa81220b68d997d012f8f05c49ab3152c1a2a46144830f516339b11c206d3b9b",
      "e6b7fedd06fe0ff0b9e669156f86f0d9d80f217a0979c72cfc7670bdcc4c0c9a",
  };
  constexpr std::array<std::string_view, 4> kRejectedStationaryHashes{
      "b863ad06a34cc6fefa5e4beca274f40033e59305d50a2c3dc31cd142d717f3a6",
      "3420cdb9e7c02dbbfd0bd2aec2c837583c8e3fedfaff26820beebca43be47461",
      "fa81220b68d997d012f8f05c49ab3152c1a2a46144830f516339b11c206d3b9b",
      "e6b7fedd06fe0ff0b9e669156f86f0d9d80f217a0979c72cfc7670bdcc4c0c9a",
  };
  constexpr std::array<std::string_view, 4>
      kCrossedRejectedStationaryHashes{
          "b863ad06a34cc6fefa5e4beca274f40033e59305d50a2c3dc31cd142d717f3a6",
          "3420cdb9e7c02dbbfd0bd2aec2c837583c8e3fedfaff26820beebca43be47461",
          "b6004812d4dc668d8cfd444272caebc67db463febc0fb10548510dcf24b4b206",
          "72d71d27cb4ce2f4292b66e486ffb4eb4e5b1b52059e16e6872674149dd3a113",
      };
  constexpr std::array<std::string_view, 4> kRankDeficientHashes{
      "6e19731100aa60b8e3289b80e2dceaf1c2af88b15b96c88ea37ddfba73787c74",
      "d068910a977411708181dc59cfcd7a49b3014598d3a9faee18c1e2128e20396d",
      "3493ec907d56aea56dc6b1c2aef0da174d1895ae71163a092f598a44fcd5b542",
      "37500a0cdca9a778b2830994e46a393cb148bdd5a3c1d36dd120c66c7a964c12",
  };
  constexpr std::array<std::string_view, 4> kIllConditionedHashes{
      "6e19731100aa60b8e3289b80e2dceaf1c2af88b15b96c88ea37ddfba73787c74",
      "d068910a977411708181dc59cfcd7a49b3014598d3a9faee18c1e2128e20396d",
      "3493ec907d56aea56dc6b1c2aef0da174d1895ae71163a092f598a44fcd5b542",
      "82411f9c40773a21b4dfa61d99042aceda562ffef4d1df8d865de1bdc17a7cdf",
  };
  constexpr std::array<std::string_view, 4> kObservabilityHashes{
      "2f382799f515feb9973dce4d8e14c4e764b5f30a353fd750ca11a1ea05c31bfc",
      "59402956f862c9a0dbc68a5e798126f0a639e17efe1431233d2cb712d5b103db",
      "924fca9311f1d78a419b1438d0761785393a8308f3de1e18751244c0eb6ca6b0",
      "23c3898895be0e0a10b0a36ecd8ddac1a3e65fc434e0a188e8293accd759d937",
  };

  [[nodiscard]] std::string readFile( const std::filesystem::path& path )
  {
    std::ifstream      in( path, std::ios::binary );
    std::ostringstream bytes;
    bytes << in.rdbuf();
    return bytes.str();
  }

  void writeFile( const std::filesystem::path& path, std::string_view bytes )
  {
    std::ofstream out( path, std::ios::binary );
    ASSERT_TRUE( out.is_open() ) << path;
    out.write( bytes.data(), static_cast<std::streamsize>( bytes.size() ) );
    ASSERT_TRUE( out.good() ) << path;
  }

  [[nodiscard]] std::string sha256( std::string_view bytes )
  {
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    std::size_t                                digest_size = digest.size();
    const int                                  ok          = EVP_Q_digest( nullptr, "SHA256", nullptr, bytes.data(),
                                                                           bytes.size(), digest.data(), &digest_size );
    EXPECT_EQ( ok, 1 );
    EXPECT_EQ( digest_size, 32U );

    std::ostringstream hex;
    hex << std::hex << std::setfill( '0' );
    for ( std::size_t i = 0; i < digest_size; ++i )
    {
      hex << std::setw( 2 ) << static_cast<unsigned int>( digest[ i ] );
    }
    return hex.str();
  }

  [[nodiscard]] std::string sha256File( const std::filesystem::path& path )
  {
    return sha256( readFile( path ) );
  }

  void toggleFirstHexDigit( std::string& value )
  {
    ASSERT_FALSE( value.empty() );
    const char before = value.front();
    value.front()     = before == '0' ? '1' : '0';
    ASSERT_NE( value.front(), before );
  }

  [[nodiscard]] FixtureBytes firstZeroFixture()
  {
    return FixtureBytes{
        {
            "0.000000000 0 0 0 0 0 0 1\n",
            std::string{ kDiagHeader } +
                "0,ok,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0\n",
            std::string{ kPacketHeader } +
                "0,0,0,0,0,0,0,0,first_zero\n",
            std::string{ kSampleHeader },
        },
        kFirstZeroHashes,
    };
  }

  [[nodiscard]] FixtureBytes stationaryFixture( std::string_view                       fit_gx,
                                                std::string_view                       fit_gy,
                                                std::string_view                       fit_gz,
                                                const std::array<std::string_view, 4>& hashes,
                                                bool                                   mixed_validation = false,
                                                std::uint64_t                          last_second      = 160U,
                                                std::optional<std::uint64_t>           rejected_second  = std::nullopt )
  {
    FixtureBytes       fixture;
    std::ostringstream tum;
    std::ostringstream diag;
    std::ostringstream packets;
    std::ostringstream samples;
    diag << kDiagHeader;
    packets << kPacketHeader << "0,0,0,0,0,0,0,0,first_zero\n";
    samples << kSampleHeader;
    for ( std::uint64_t i = 0; i <= last_second; ++i )
    {
      const auto timestamp_ns = i * 1'000'000'000ULL;
      const bool rejected     = rejected_second.has_value() &&
                            i == *rejected_second;
      if ( !rejected )
      {
        tum << i << ".000000000 0 0 0 0 0 0 1\n";
      }
      diag << timestamp_ns << ( rejected ? ",rejected" : ",ok" )
           << ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0\n";
      if ( i == 0U )
      {
        continue;
      }
      packets << i << ',' << ( timestamp_ns - 1'000'000'000ULL ) << ','
              << timestamp_ns << ",0,0,2,1000000000,1000000000,valid\n";
      const bool             zero_validation = mixed_validation && i > 140U;
      const std::string_view gx              = zero_validation ? "0" : fit_gx;
      const std::string_view gy              = zero_validation ? "0" : fit_gy;
      const std::string_view gz              = zero_validation ? "0" : fit_gz;
      samples << i << ",0," << ( timestamp_ns - 1'000'000'000ULL ) << ','
              << gx << ',' << gy << ',' << gz << '\n';
      samples << i << ",1," << timestamp_ns << ',' << gx << ',' << gy
              << ',' << gz << '\n';
    }
    fixture.files  = { tum.str(), diag.str(), packets.str(), samples.str() };
    fixture.hashes = hashes;
    return fixture;
  }

  [[nodiscard]] std::string tumTimestamp( std::int64_t timestamp_ns )
  {
    std::ostringstream value;
    value << timestamp_ns / 1'000'000'000 << '.' << std::setw( 9 )
          << std::setfill( '0' ) << timestamp_ns % 1'000'000'000;
    return value.str();
  }

  void appendIdentityEndpoint( std::ostringstream& tum,
                               std::ostringstream& diag,
                               std::int64_t        timestamp_ns )
  {
    tum << tumTimestamp( timestamp_ns ) << " 0 0 0 0 0 0 1\n";
    diag << timestamp_ns
         << ",ok,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0\n";
  }

  [[nodiscard]] FixtureBytes degenerateFitFixture(
      std::string_view                       rate_x,
      const std::array<std::string_view, 4>& hashes )
  {
    FixtureBytes       fixture;
    std::ostringstream tum;
    std::ostringstream diag;
    std::ostringstream packets;
    std::ostringstream samples;
    diag << kDiagHeader;
    packets << kPacketHeader << "0,0,0,0,0,0,0,0,first_zero\n";
    samples << kSampleHeader;
    appendIdentityEndpoint( tum, diag, 0 );
    std::uint64_t packet_index = 1U;
    std::int64_t  timestamp_ns = 0;
    for ( std::size_t half = 0; half < 2U; ++half )
    {
      for ( std::size_t i = 0; i < 2'250U; ++i )
      {
        const auto next_ns = timestamp_ns + 20'000'000;
        appendIdentityEndpoint( tum, diag, next_ns );
        packets << packet_index << ',' << timestamp_ns << ',' << next_ns
                << ",0,0,2,20000000,20000000,valid\n";
        samples << packet_index << ",0," << timestamp_ns << ',' << rate_x
                << ",0,0\n";
        samples << packet_index << ",1," << next_ns << ',' << rate_x
                << ",0,0\n";
        ++packet_index;
        timestamp_ns = next_ns;
      }
      const auto boundary_ns =
          static_cast<std::int64_t>( half + 1U ) * 50'000'000'000LL;
      appendIdentityEndpoint( tum, diag, boundary_ns );
      packets << packet_index << ',' << timestamp_ns << ',' << boundary_ns
              << ",0,1,0,0," << boundary_ns - timestamp_ns << ",gap\n";
      ++packet_index;
      timestamp_ns = boundary_ns;
    }
    fixture.files  = { tum.str(), diag.str(), packets.str(), samples.str() };
    fixture.hashes = hashes;
    return fixture;
  }

  [[nodiscard]] FixtureBytes observabilityFixture()
  {
    FixtureBytes       fixture;
    std::ostringstream tum;
    std::ostringstream diag;
    std::ostringstream packets;
    std::ostringstream samples;
    diag << kDiagHeader;
    packets << kPacketHeader << "0,0,0,0,0,0,0,0,first_zero\n";
    samples << kSampleHeader;
    for ( std::uint64_t second = 0; second <= 160U; ++second )
    {
      std::string_view qx = "0";
      std::string_view qw = "1";
      if ( second == 1U )
      {
        qx = "0.99999999995";
        qw = "9.9999999999600785e-06";
      }
      else if ( ( second >= 3U && second <= 50U ) || second == 52U )
      {
        qx = "-0.0004999999791666669";
        qw = "0.99999987500000265";
      }
      else if ( second == 51U )
      {
        qx = "0.99999986995000278";
        qw = "0.00050999997789157199";
      }
      else if ( second >= 53U )
      {
        qx = "-0.00099999983333334168";
        qw = "0.99999950000004167";
      }

      const auto timestamp_ns =
          static_cast<std::int64_t>( second ) * 1'000'000'000;
      tum << second << ".000000000 0 0 0 " << qx << " 0 0 " << qw
          << '\n';
      diag << timestamp_ns
           << ",ok,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0\n";
      if ( second == 0U )
      {
        continue;
      }
      const auto t_prev_ns = timestamp_ns - 1'000'000'000;
      packets << second << ',' << t_prev_ns << ',' << timestamp_ns
              << ",0,0,2,1000000000,1000000000,valid\n";
      samples << second << ",0," << t_prev_ns << ",0,0,0\n";
      samples << second << ",1," << timestamp_ns << ",0,0,0\n";
    }
    fixture.files  = { tum.str(), diag.str(), packets.str(), samples.str() };
    fixture.hashes = kObservabilityHashes;
    return fixture;
  }

  [[nodiscard]] GyroAlignmentProtocolDescriptor syntheticDescriptor(
      const FixtureBytes& fixture )
  {
    std::vector<GyroAlignmentInputIdentity> inputs;
    inputs.reserve( kInputNames.size() );
    for ( std::size_t i = 0; i < kInputNames.size(); ++i )
    {
      inputs.push_back( GyroAlignmentInputIdentity{
          std::string{ kInputNames[ i ] }, std::string{ fixture.hashes[ i ] } } );
    }
    return GyroAlignmentProtocolDescriptor{
        .protocol_id       = std::string{ kSyntheticProtocolId },
        .inputs            = std::move( inputs ),
        .protocol_identity = {
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            "cccccccccccccccccccccccccccccccccccccccc",
            "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd" },
        .analyzer_identity = { "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee", "ffffffffffffffffffffffffffffffffffffffff", "Synthetic", "tester", "1.0" },
        .provenance        = { "synthetic-q1-run", "1111111111111111111111111111111111111111", "2222222222222222222222222222222222222222", "3333333333333333333333333333333333333333333333333333333333333333", "4444444444444444444444444444444444444444444444444444444444444444", "deadbeef", "5555555555555555555555555555555555555555555555555555555555555555", "6666666666666666666666666666666666666666", "7777777777777777777777777777777777777777" },
    };
  }

  [[nodiscard]] std::filesystem::path materializeFixture(
      const std::filesystem::path& root, const FixtureBytes& fixture )
  {
    const auto q1_dir = root / "q1";
    std::filesystem::create_directory( q1_dir );
    for ( std::size_t i = 0; i < kInputNames.size(); ++i )
    {
      writeFile( q1_dir / kInputNames[ i ], fixture.files[ i ] );
    }
    return q1_dir;
  }

  [[nodiscard]] std::vector<std::string> keys(
      const nlohmann::ordered_json& object )
  {
    std::vector<std::string> result;
    for ( auto it = object.begin(); it != object.end(); ++it )
    {
      result.push_back( it.key() );
    }
    return result;
  }

  [[nodiscard]] std::vector<std::string> directoryEntries(
      const std::filesystem::path& path )
  {
    std::vector<std::string> entries;
    if ( !std::filesystem::exists( path ) )
    {
      return entries;
    }
    for ( const auto& entry : std::filesystem::directory_iterator{ path } )
    {
      entries.push_back( entry.path().filename().string() );
    }
    std::sort( entries.begin(), entries.end() );
    return entries;
  }

#if defined( __linux__ ) && defined( __x86_64__ )
  struct SyscallTrace
  {
    struct OpenedFile
    {
      std::string    path;
      std::uintmax_t device = 0U;
      std::uintmax_t inode  = 0U;
    };

    std::vector<OpenedFile>  opened_files;
    std::vector<std::string> rename_attempts;
    std::vector<std::string> renamed_paths;
    std::vector<std::string> unlinked_paths;
    std::vector<std::string> probed_paths;
    int                      child_exit     = -1;
    bool                     fault_injected = false;
  };

  enum class TracedPathKind
  {
    kNone,
    kOpen,
    kRename,
    kUnlink,
    kProbe,
  };

  struct PendingSyscall
  {
    TracedPathKind kind = TracedPathKind::kNone;
    std::string    path;
    int            dir_fd   = AT_FDCWD;
    bool           injected = false;
  };

  struct TraceFault
  {
    std::string rename_path;
  };

  class Tracee
  {
  public:
    explicit Tracee( pid_t pid ) : m_pid{ pid } {}

    ~Tracee()
    {
      if ( !m_owned )
      {
        return;
      }
      static_cast<void>( ::kill( m_pid, SIGKILL ) );
      int status = 0;
      while ( ::waitpid( m_pid, &status, 0 ) < 0 && errno == EINTR )
      {
      }
    }

    Tracee( const Tracee& )            = delete;
    Tracee& operator=( const Tracee& ) = delete;

    void release() noexcept { m_owned = false; }

  private:
    pid_t m_pid;
    bool  m_owned = true;
  };

  [[nodiscard]] pid_t waitNoIntr( pid_t pid, int& status )
  {
    pid_t waited = -1;
    do
    {
      waited = ::waitpid( pid, &status, 0 );
    } while ( waited < 0 && errno == EINTR );
    return waited;
  }

  [[nodiscard]] std::optional<std::string> readLink(
      const std::filesystem::path& path )
  {
    std::array<char, 4096> buffer{};
    const ssize_t          count =
        ::readlink( path.c_str(), buffer.data(), buffer.size() - 1U );
    if ( count < 0 )
    {
      return std::nullopt;
    }
    return std::string{ buffer.data(), static_cast<std::size_t>( count ) };
  }

  [[nodiscard]] std::string normalizeChildPath( pid_t            pid,
                                                int              dir_fd,
                                                std::string_view raw_path )
  {
    const std::filesystem::path raw{ raw_path };
    if ( raw.is_absolute() )
    {
      return raw.lexically_normal().string();
    }
    const auto proc_root = std::filesystem::path{ "/proc" } /
                           std::to_string( pid );
    const auto base = dir_fd == AT_FDCWD
                          ? readLink( proc_root / "cwd" )
                          : readLink( proc_root / "fd" /
                                      std::to_string( dir_fd ) );
    if ( !base.has_value() )
    {
      return raw.lexically_normal().string();
    }
    return ( std::filesystem::path{ *base } / raw ).lexically_normal().string();
  }

  [[nodiscard]] std::string readChildString( pid_t         pid,
                                             unsigned long address )
  {
    constexpr std::size_t kMaxPathBytes = 4096U;
    std::string           value;
    value.reserve( 128U );
    while ( value.size() < kMaxPathBytes )
    {
      errno           = 0;
      const long word = ::ptrace( PTRACE_PEEKDATA, pid,
                                  reinterpret_cast<void*>( address ),
                                  nullptr );
      if ( word == -1 && errno != 0 )
      {
        throw std::system_error{ errno, std::generic_category(),
                                 "ptrace PEEKDATA" };
      }
      const auto* bytes = reinterpret_cast<const unsigned char*>( &word );
      for ( std::size_t i = 0; i < sizeof( word ); ++i )
      {
        if ( bytes[ i ] == 0U )
        {
          return value;
        }
        value.push_back( static_cast<char>( bytes[ i ] ) );
      }
      address += sizeof( word );
    }
    throw std::runtime_error{ "traced path exceeds PATH_MAX" };
  }

  [[nodiscard]] PendingSyscall makePending( pid_t          pid,
                                            TracedPathKind kind,
                                            int            dir_fd,
                                            unsigned long  address )
  {
    const auto raw_path = readChildString( pid, address );
    return { kind, normalizeChildPath( pid, dir_fd, raw_path ), dir_fd,
             false };
  }

  [[nodiscard]] PendingSyscall syscallEntry( pid_t                   pid,
                                             const user_regs_struct& regs )
  {
    const auto nr = static_cast<long>( regs.orig_rax );
    if ( nr == SYS_open )
    {
      return makePending( pid, TracedPathKind::kOpen, AT_FDCWD,
                          regs.rdi );
    }
    if ( nr == SYS_openat
#ifdef SYS_openat2
         || nr == SYS_openat2
#endif
    )
    {
      return makePending( pid, TracedPathKind::kOpen,
                          static_cast<int>( regs.rdi ), regs.rsi );
    }
    if ( nr == SYS_rename )
    {
      return makePending( pid, TracedPathKind::kRename, AT_FDCWD,
                          regs.rsi );
    }
    if ( nr == SYS_renameat
#ifdef SYS_renameat2
         || nr == SYS_renameat2
#endif
    )
    {
      return makePending( pid, TracedPathKind::kRename,
                          static_cast<int>( regs.rdx ), regs.r10 );
    }
    if ( nr == SYS_unlink )
    {
      return makePending( pid, TracedPathKind::kUnlink, AT_FDCWD,
                          regs.rdi );
    }
    if ( nr == SYS_unlinkat )
    {
      return makePending( pid, TracedPathKind::kUnlink,
                          static_cast<int>( regs.rdi ), regs.rsi );
    }
#ifdef SYS_newfstatat
    if ( nr == SYS_newfstatat )
    {
      return makePending( pid, TracedPathKind::kProbe,
                          static_cast<int>( regs.rdi ), regs.rsi );
    }
#endif
#ifdef SYS_statx
    if ( nr == SYS_statx )
    {
      return makePending( pid, TracedPathKind::kProbe,
                          static_cast<int>( regs.rdi ), regs.rsi );
    }
#endif
#ifdef SYS_stat
    if ( nr == SYS_stat )
    {
      return makePending( pid, TracedPathKind::kProbe, AT_FDCWD,
                          regs.rdi );
    }
#endif
#ifdef SYS_lstat
    if ( nr == SYS_lstat )
    {
      return makePending( pid, TracedPathKind::kProbe, AT_FDCWD,
                          regs.rdi );
    }
#endif
    if ( nr == SYS_access )
    {
      return makePending( pid, TracedPathKind::kProbe, AT_FDCWD,
                          regs.rdi );
    }
#ifdef SYS_faccessat
    if ( nr == SYS_faccessat )
    {
      return makePending( pid, TracedPathKind::kProbe,
                          static_cast<int>( regs.rdi ), regs.rsi );
    }
#endif
#ifdef SYS_faccessat2
    if ( nr == SYS_faccessat2 )
    {
      return makePending( pid, TracedPathKind::kProbe,
                          static_cast<int>( regs.rdi ), regs.rsi );
    }
#endif
    if ( nr == SYS_mkdir )
    {
      return makePending( pid, TracedPathKind::kProbe, AT_FDCWD,
                          regs.rdi );
    }
#ifdef SYS_mkdirat
    if ( nr == SYS_mkdirat )
    {
      return makePending( pid, TracedPathKind::kProbe,
                          static_cast<int>( regs.rdi ), regs.rsi );
    }
#endif
    return {};
  }

  void recordSuccessfulSyscall( pid_t                 pid,
                                const PendingSyscall& pending,
                                long                  return_value,
                                SyscallTrace&         trace )
  {
    switch ( pending.kind )
    {
      case TracedPathKind::kOpen:
      {
        struct stat info
        {
        };
        const auto fd_path = std::filesystem::path{ "/proc" } /
                             std::to_string( pid ) / "fd" /
                             std::to_string( return_value );
        if ( ::stat( fd_path.c_str(), &info ) != 0 )
        {
          throw std::system_error{ errno, std::generic_category(),
                                   "stat traced fd" };
        }
        trace.opened_files.push_back(
            { pending.path, static_cast<std::uintmax_t>( info.st_dev ),
              static_cast<std::uintmax_t>( info.st_ino ) } );
        break;
      }
      case TracedPathKind::kRename:
        trace.renamed_paths.push_back( pending.path );
        break;
      case TracedPathKind::kUnlink:
        trace.unlinked_paths.push_back( pending.path );
        break;
      case TracedPathKind::kProbe:
      case TracedPathKind::kNone:
        break;
    }
  }

  [[nodiscard]] SyscallTrace traceChildSyscalls(
      const std::function<int()>& child,
      std::optional<TraceFault>   fault = std::nullopt )
  {
    const pid_t pid = ::fork();
    if ( pid < 0 )
    {
      throw std::system_error{ errno, std::generic_category(), "fork" };
    }
    if ( pid == 0 )
    {
      if ( ::ptrace( PTRACE_TRACEME, 0, nullptr, nullptr ) < 0 )
      {
        ::_exit( 125 );
      }
      ::raise( SIGSTOP );
      ::_exit( child() );
    }

    Tracee tracee{ pid };

    int status = 0;
    if ( waitNoIntr( pid, status ) != pid )
    {
      throw std::system_error{ errno, std::generic_category(),
                               "initial waitpid" };
    }
    if ( WIFEXITED( status ) || WIFSIGNALED( status ) )
    {
      tracee.release();
      throw std::runtime_error{ "ptrace child exited before initial stop" };
    }
    if ( !WIFSTOPPED( status ) )
    {
      throw std::runtime_error{ "ptrace child did not enter initial stop" };
    }
    constexpr long kPtraceOptions = PTRACE_O_TRACESYSGOOD | PTRACE_O_EXITKILL;
    if ( ::ptrace( PTRACE_SETOPTIONS, pid, nullptr, kPtraceOptions ) < 0 ||
         ::ptrace( PTRACE_SYSCALL, pid, nullptr, nullptr ) < 0 )
    {
      throw std::system_error{ errno, std::generic_category(),
                               "ptrace setup" };
    }

    SyscallTrace   trace;
    PendingSyscall pending;
    bool           entering = true;
    while ( true )
    {
      if ( waitNoIntr( pid, status ) != pid )
      {
        throw std::system_error{ errno, std::generic_category(), "waitpid" };
      }
      if ( WIFEXITED( status ) )
      {
        trace.child_exit = WEXITSTATUS( status );
        tracee.release();
        return trace;
      }
      if ( WIFSIGNALED( status ) )
      {
        tracee.release();
        throw std::runtime_error{ "ptrace child terminated by signal" };
      }

      int signal = 0;
      if ( WSTOPSIG( status ) == ( SIGTRAP | 0x80 ) )
      {
        user_regs_struct regs{};
        if ( ::ptrace( PTRACE_GETREGS, pid, nullptr, &regs ) < 0 )
        {
          throw std::system_error{ errno, std::generic_category(),
                                   "ptrace GETREGS" };
        }
        if ( entering )
        {
          pending = syscallEntry( pid, regs );
          if ( pending.kind != TracedPathKind::kNone )
          {
            trace.probed_paths.push_back( pending.path );
          }
          if ( pending.kind == TracedPathKind::kRename )
          {
            trace.rename_attempts.push_back( pending.path );
          }
          if ( fault.has_value() && !trace.fault_injected &&
               pending.kind == TracedPathKind::kRename &&
               pending.path == fault->rename_path )
          {
            regs.orig_rax        = static_cast<unsigned long>( -1LL );
            pending.injected     = true;
            trace.fault_injected = true;
            if ( ::ptrace( PTRACE_SETREGS, pid, nullptr, &regs ) < 0 )
            {
              throw std::system_error{ errno, std::generic_category(),
                                       "ptrace SETREGS entry" };
            }
          }
        }
        else if ( pending.injected )
        {
          regs.rax = static_cast<unsigned long>( -EIO );
          if ( ::ptrace( PTRACE_SETREGS, pid, nullptr, &regs ) < 0 )
          {
            throw std::system_error{ errno, std::generic_category(),
                                     "ptrace SETREGS exit" };
          }
        }
        else if ( static_cast<std::int64_t>( regs.rax ) >= 0 )
        {
          recordSuccessfulSyscall(
              pid, pending, static_cast<long>( regs.rax ), trace );
        }
        entering = !entering;
      }
      else
      {
        signal = WSTOPSIG( status );
        if ( signal == SIGTRAP || signal == SIGSTOP )
        {
          signal = 0;
        }
      }
      if ( ::ptrace( PTRACE_SYSCALL, pid, nullptr, signal ) < 0 )
      {
        throw std::system_error{ errno, std::generic_category(),
                                 "ptrace SYSCALL" };
      }
    }
  }

  [[nodiscard]] std::vector<std::string> finalRenames(
      const SyscallTrace& trace, const std::filesystem::path& out_dir )
  {
    const std::array<std::filesystem::path, 3> finals{
        out_dir / "gyro_alignment.json",
        out_dir / "gyro_alignment_blocks.csv",
        out_dir / "gyro_alignment.manifest.json",
    };
    std::vector<std::string> renamed;
    for ( const auto& path : trace.renamed_paths )
    {
      if ( std::any_of( finals.begin(), finals.end(), [ & ]( const auto& final ) {
             return final.string() == path;
           } ) )
      {
        renamed.push_back( path );
      }
    }
    return renamed;
  }

  [[nodiscard]] bool validPublishTrace(
      const SyscallTrace& trace, const std::filesystem::path& out_dir )
  {
    return finalRenames( trace, out_dir ) ==
           std::vector<std::string>{
               ( out_dir / "gyro_alignment.json" ).string(),
               ( out_dir / "gyro_alignment_blocks.csv" ).string(),
               ( out_dir / "gyro_alignment.manifest.json" ).string(),
           };
  }

  [[nodiscard]] bool validRollbackTrace(
      const SyscallTrace& trace, const std::filesystem::path& out_dir )
  {
    const auto finals = finalRenames( trace, out_dir );
    for ( const auto& path : finals )
    {
      if ( std::find( trace.unlinked_paths.begin(), trace.unlinked_paths.end(),
                      path ) == trace.unlinked_paths.end() )
      {
        return false;
      }
    }
    return std::find( finals.begin(), finals.end(),
                      ( out_dir / "gyro_alignment.manifest.json" ).string() ) ==
           finals.end();
  }

  [[nodiscard]] std::pair<std::uintmax_t, std::uintmax_t> fileIdentity(
      const std::filesystem::path& path )
  {
    struct stat info
    {
    };
    if ( ::stat( path.c_str(), &info ) != 0 )
    {
      throw std::system_error{ errno, std::generic_category(), "stat input" };
    }
    return { static_cast<std::uintmax_t>( info.st_dev ),
             static_cast<std::uintmax_t>( info.st_ino ) };
  }

  [[nodiscard]] bool pathIsWithin( const std::string&           candidate,
                                   const std::filesystem::path& root )
  {
    const auto normalized_candidate =
        std::filesystem::path{ candidate }.lexically_normal();
    const auto normalized_root = root.lexically_normal();
    if ( normalized_candidate == normalized_root )
    {
      return true;
    }
    const auto relative = normalized_candidate.lexically_relative(
        normalized_root );
    return !relative.empty() && relative.begin()->string() != "..";
  }

  void expectNoFilesystemProbe( const SyscallTrace&          trace,
                                const std::filesystem::path& q1_dir,
                                const std::filesystem::path& out_dir )
  {
    for ( const auto& path : trace.probed_paths )
    {
      EXPECT_FALSE( pathIsWithin( path, q1_dir ) ) << path;
      EXPECT_FALSE( pathIsWithin( path, out_dir ) ) << path;
    }
  }
#endif

  void expectHard( const GyroAlignmentResult& result,
                   GyroAlignmentErrorCode     code )
  {
    EXPECT_EQ( result.verdict.status, GyroAlignmentStatus::kHardError );
    EXPECT_EQ( result.verdict.exit_code, 1 );
    EXPECT_EQ( result.error, code );
    EXPECT_FALSE( result.analysis.has_value() );
    EXPECT_FALSE( result.verdict.detail.empty() );
  }

  void expectInvalidDescriptorPreFilesystem(
      GyroAlignmentProtocolDescriptor descriptor,
      const std::filesystem::path&    q1_dir,
      const std::filesystem::path&    out_dir )
  {
#if defined( __linux__ ) && defined( __x86_64__ )
    const auto trace = traceChildSyscalls( [ & ] {
      const auto result =
          runGyroAlignment( std::move( descriptor ), q1_dir, out_dir );
      return result.verdict.status == GyroAlignmentStatus::kHardError &&
                     result.verdict.exit_code == 1 &&
                     result.error ==
                         GyroAlignmentErrorCode::kInvalidProtocolDescriptor &&
                     !result.analysis.has_value() &&
                     !result.verdict.detail.empty() &&
                     result.error != GyroAlignmentErrorCode::kInputHashMismatch
                 ? 0
                 : 1;
    } );
    ASSERT_EQ( trace.child_exit, 0 );
    expectNoFilesystemProbe( trace, q1_dir, out_dir );
#else
    const auto result =
        runGyroAlignment( std::move( descriptor ), q1_dir, out_dir );
    expectHard( result, GyroAlignmentErrorCode::kInvalidProtocolDescriptor );
#endif
    EXPECT_FALSE( std::filesystem::exists( q1_dir ) );
    EXPECT_FALSE( std::filesystem::exists( out_dir ) );
  }

  [[nodiscard]] int runCommand( const std::string&           args,
                                const std::filesystem::path& stdout_path,
                                const std::filesystem::path& stderr_path )
  {
    const std::string command =
        std::string{ "\"" PHAD_GYRO_ALIGN_PATH "\" " } + args + " > \"" +
        stdout_path.string() + "\" 2> \"" + stderr_path.string() + "\"";
    const int status = std::system( command.c_str() );
#ifdef __linux__
    if ( status == -1 || !WIFEXITED( status ) )
    {
      return -1;
    }
    return WEXITSTATUS( status );
#else
    return status;
#endif
  }

  [[nodiscard]] bool qualificationConsumerAccepts(
      const nlohmann::ordered_json& manifest )
  {
    return manifest.is_object() &&
           manifest.value( "schema_version", "" ) ==
               "phad.gyro_alignment.manifest.v1" &&
           manifest.value( "protocol_id", "" ) ==
               "PHAD-M4-Q3-GYRO-ALIGN-V1";
  }

  void expectNoOutcomeKeys( const nlohmann::ordered_json& value )
  {
    if ( value.is_array() )
    {
      for ( const auto& item : value )
      {
        expectNoOutcomeKeys( item );
      }
      return;
    }
    if ( !value.is_object() )
    {
      return;
    }
    for ( auto it = value.begin(); it != value.end(); ++it )
    {
      std::string key = it.key();
      std::transform( key.begin(), key.end(), key.begin(),
                      []( unsigned char c ) {
                        return static_cast<char>( std::tolower( c ) );
                      } );
      EXPECT_NE( key, "groundtruth" );
      EXPECT_NE( key, "ground_truth" );
      EXPECT_NE( key, "ate_rmse" );
      EXPECT_NE( key, "dataset_locator" );
      EXPECT_NE( key.rfind( "rpe_", 0U ), 0U );
      expectNoOutcomeKeys( it.value() );
    }
  }

  void expectProtocolIdentity(
      const nlohmann::ordered_json&        value,
      const GyroAlignmentProtocolIdentity& expected )
  {
    EXPECT_EQ( value.at( "design_commit" ), expected.design_commit );
    EXPECT_EQ( value.at( "design_tree" ), expected.design_tree );
    EXPECT_EQ( value.at( "design_blob" ), expected.design_blob );
    EXPECT_EQ( value.at( "design_sha256" ), expected.design_sha256 );
  }

  void expectAnalyzerIdentity(
      const nlohmann::ordered_json&        value,
      const GyroAlignmentAnalyzerIdentity& expected )
  {
    EXPECT_EQ( value.at( "source_commit" ), expected.source_commit );
    EXPECT_EQ( value.at( "source_tree" ), expected.source_tree );
    EXPECT_EQ( value.at( "build_type" ), expected.build_type );
    EXPECT_EQ( value.at( "compiler" ), expected.compiler );
    EXPECT_EQ( value.at( "compiler_version" ), expected.compiler_version );
  }

  void expectProvenance( const nlohmann::ordered_json&  value,
                         const GyroAlignmentProvenance& expected )
  {
    EXPECT_EQ( value.at( "q1_source_run" ), expected.q1_source_run );
    EXPECT_EQ( value.at( "q1_source_commit" ), expected.q1_source_commit );
    EXPECT_EQ( value.at( "q1_git_tree_object" ),
               expected.q1_git_tree_object );
    EXPECT_EQ( value.at( "q1_git_ls_tree_sha256" ),
               expected.q1_git_ls_tree_sha256 );
    EXPECT_EQ( value.at( "q1_meta_sha256" ), expected.q1_meta_sha256 );
    EXPECT_EQ( value.at( "q1_config_hash" ), expected.q1_config_hash );
    EXPECT_EQ( value.at( "q1_input_manifest_v2_sha256" ),
               expected.q1_input_manifest_v2_sha256 );
    EXPECT_EQ( value.at( "q2_commit" ), expected.q2_commit );
    EXPECT_EQ( value.at( "q2_tree" ), expected.q2_tree );
  }

  [[nodiscard]] std::string_view fitStatusName(
      GyroAlignmentFitStatus status )
  {
    switch ( status )
    {
      case GyroAlignmentFitStatus::kSupportInsufficient:
        return "support_insufficient";
      case GyroAlignmentFitStatus::kRankDeficient:
        return "rank_deficient";
      case GyroAlignmentFitStatus::kIllConditioned:
        return "ill_conditioned";
      case GyroAlignmentFitStatus::kObservabilityFailed:
        return "observability_failed";
      case GyroAlignmentFitStatus::kSolved:
        return "solved";
    }
    throw std::logic_error{ "unknown fit status" };
  }

  template <typename OptionalVector>
  void expectOptionalVector( const nlohmann::ordered_json& value,
                             const OptionalVector&         expected )
  {
    if ( !expected.has_value() )
    {
      EXPECT_TRUE( value.is_null() );
      return;
    }
    ASSERT_TRUE( value.is_array() );
    ASSERT_EQ( value.size(), 3U );
    EXPECT_DOUBLE_EQ( value.at( 0 ).get<double>(), expected->x() );
    EXPECT_DOUBLE_EQ( value.at( 1 ).get<double>(), expected->y() );
    EXPECT_DOUBLE_EQ( value.at( 2 ).get<double>(), expected->z() );
  }

  template <typename T>
  void expectOptionalScalar( const nlohmann::ordered_json& value,
                             const std::optional<T>&       expected )
  {
    if ( !expected.has_value() )
    {
      EXPECT_TRUE( value.is_null() );
      return;
    }
    if constexpr ( std::is_same_v<T, double> )
    {
      EXPECT_DOUBLE_EQ( value.get<double>(), *expected );
    }
    else
    {
      EXPECT_EQ( value.get<T>(), *expected );
    }
  }

  void expectFitArtifactSchema( const nlohmann::ordered_json& value,
                                GyroAlignmentFitStatus        status )
  {
    ASSERT_TRUE( value.is_object() );
    EXPECT_EQ( keys( value ),
               std::vector<std::string>( kFitArtifactKeys.begin(),
                                         kFitArtifactKeys.end() ) );
    EXPECT_EQ( value.at( "status" ), fitStatusName( status ) );

    const auto expect_null = [ & ]( std::string_view name ) {
      EXPECT_TRUE( value.at( std::string{ name } ).is_null() ) << name;
    };
    const auto expect_present = [ & ]( std::string_view name ) {
      EXPECT_FALSE( value.at( std::string{ name } ).is_null() ) << name;
    };

    expect_present( "status" );
    expect_present( "eligible_duration_ns" );
    expect_present( "packet_count" );
    switch ( status )
    {
      case GyroAlignmentFitStatus::kSupportInsufficient:
        expect_null( "bias_radps" );
        expect_null( "singular_values" );
        expect_null( "rank" );
        expect_null( "condition" );
        expect_null( "loss_zero_rad2" );
        expect_null( "loss_fit_rad2" );
        expect_null( "nonlinear_nonincrease" );
        break;
      case GyroAlignmentFitStatus::kRankDeficient:
        expect_null( "bias_radps" );
        expect_present( "singular_values" );
        expect_present( "rank" );
        expect_null( "condition" );
        expect_null( "loss_zero_rad2" );
        expect_null( "loss_fit_rad2" );
        expect_null( "nonlinear_nonincrease" );
        break;
      case GyroAlignmentFitStatus::kIllConditioned:
      case GyroAlignmentFitStatus::kObservabilityFailed:
        expect_null( "bias_radps" );
        expect_present( "singular_values" );
        expect_present( "rank" );
        expect_present( "condition" );
        expect_null( "loss_zero_rad2" );
        expect_null( "loss_fit_rad2" );
        expect_null( "nonlinear_nonincrease" );
        if ( !value.at( "rank" ).is_null() )
        {
          EXPECT_EQ( value.at( "rank" ).get<std::uint64_t>(), 3U );
        }
        break;
      case GyroAlignmentFitStatus::kSolved:
        expect_present( "bias_radps" );
        expect_present( "singular_values" );
        expect_present( "rank" );
        expect_present( "condition" );
        expect_present( "loss_zero_rad2" );
        expect_present( "loss_fit_rad2" );
        expect_present( "nonlinear_nonincrease" );
        break;
    }
  }

  void expectFitArtifact( const nlohmann::ordered_json& value,
                          GyroAlignmentFitStatus        status,
                          const GyroAlignmentFit&       expected )
  {
    EXPECT_EQ( expected.status, status );
    expectFitArtifactSchema( value, status );
    EXPECT_EQ( value.at( "status" ), fitStatusName( expected.status ) );
    expectOptionalVector( value.at( "bias_radps" ), expected.bias_radps );
    expectOptionalVector( value.at( "singular_values" ),
                          expected.singular_values );
    expectOptionalScalar( value.at( "rank" ), expected.rank );
    expectOptionalScalar( value.at( "condition" ), expected.condition );
    EXPECT_EQ( value.at( "eligible_duration_ns" ).get<std::int64_t>(),
               expected.eligible_duration_ns );
    EXPECT_EQ( value.at( "packet_count" ).get<std::uint64_t>(),
               expected.packet_count );
    expectOptionalScalar( value.at( "loss_zero_rad2" ),
                          expected.loss_zero_rad2 );
    expectOptionalScalar( value.at( "loss_fit_rad2" ),
                          expected.loss_fit_rad2 );
    expectOptionalScalar( value.at( "nonlinear_nonincrease" ),
                          expected.nonlinear_nonincrease );
  }

  TEST( GyroAlignmentCliTest, FactoryMatchesIndependentFrozenV1Binding )
  {
    const GyroAlignmentProtocolDescriptor expected{
        .protocol_id = "PHAD-M4-Q3-GYRO-ALIGN-V1",
        .inputs      = {
            { "est.tum",
                   "18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321" },
            { "diag.csv",
                   "1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb" },
            { "gyro_packets.csv",
                   "fbf574545e418fc19d100b7b05f79546a5336420bca60852770605b42702a3fe" },
            { "gyro_samples.csv",
                   "da35227b40a1ab47c94217b4210b5445d11927a864d6634f8b824812ccec0344" },
        },
        .protocol_identity = { "16affcd2fc271f1db4d6a060b3d2864004d8a40b", "634b048fd9501a9c807ebfcc645f5ce9fe4d8bb3", "a6eaa88b4553cb03f012bd090d502bc3ff3576e3", "1e2f20d181af5453b383ceb09d53850045e7b68ec17b13f8d40c883d417c081b" },
        .analyzer_identity = { PHAD_Q3_EXPECTED_SOURCE_COMMIT, PHAD_Q3_EXPECTED_SOURCE_TREE, PHAD_Q3_EXPECTED_BUILD_TYPE, PHAD_Q3_EXPECTED_COMPILER, PHAD_Q3_EXPECTED_COMPILER_VERSION },
        .provenance        = { "q1_observe_final_q1_final_verifier", "74270572cc1fcc2eac82559117efd0951c800ab9", "e4379bf44db8d1127450d674b60b2e451fbe0aef", "bb5a7854e4a3a89a52c8c0332e965474b8b9b2f2d4cbe0b83bcd7224e35f94ec", "bbeaa21edaee34733f61b8ef093d45d5b8f4268fc4c0a36da261ee6664e9fab1", "402d1925", "aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5", "d1c4385809a6bf461f1c6b8acd81870f98634aa0", "9c9c991b21c4d40e8c5f2ce974334b76e1a42b64" },
    };
    const auto actual = makeGyroAlignmentV1ProtocolDescriptor();
    EXPECT_EQ( actual.protocol_id, expected.protocol_id );
    ASSERT_EQ( actual.inputs.size(), expected.inputs.size() );
    for ( std::size_t i = 0; i < expected.inputs.size(); ++i )
    {
      EXPECT_EQ( actual.inputs[ i ].basename, expected.inputs[ i ].basename );
      EXPECT_EQ( actual.inputs[ i ].expected_sha256,
                 expected.inputs[ i ].expected_sha256 );
    }
    EXPECT_EQ( actual.protocol_identity.design_commit,
               expected.protocol_identity.design_commit );
    EXPECT_EQ( actual.protocol_identity.design_tree,
               expected.protocol_identity.design_tree );
    EXPECT_EQ( actual.protocol_identity.design_blob,
               expected.protocol_identity.design_blob );
    EXPECT_EQ( actual.protocol_identity.design_sha256,
               expected.protocol_identity.design_sha256 );
    EXPECT_EQ( actual.analyzer_identity.source_commit,
               expected.analyzer_identity.source_commit );
    EXPECT_EQ( actual.analyzer_identity.source_tree,
               expected.analyzer_identity.source_tree );
    EXPECT_EQ( actual.analyzer_identity.build_type,
               expected.analyzer_identity.build_type );
    EXPECT_EQ( actual.analyzer_identity.compiler,
               expected.analyzer_identity.compiler );
    EXPECT_EQ( actual.analyzer_identity.compiler_version,
               expected.analyzer_identity.compiler_version );
    EXPECT_EQ( actual.provenance.q1_source_run,
               expected.provenance.q1_source_run );
    EXPECT_EQ( actual.provenance.q1_source_commit,
               expected.provenance.q1_source_commit );
    EXPECT_EQ( actual.provenance.q1_git_tree_object,
               expected.provenance.q1_git_tree_object );
    EXPECT_EQ( actual.provenance.q1_git_ls_tree_sha256,
               expected.provenance.q1_git_ls_tree_sha256 );
    EXPECT_EQ( actual.provenance.q1_meta_sha256,
               expected.provenance.q1_meta_sha256 );
    EXPECT_EQ( actual.provenance.q1_config_hash,
               expected.provenance.q1_config_hash );
    EXPECT_EQ( actual.provenance.q1_input_manifest_v2_sha256,
               expected.provenance.q1_input_manifest_v2_sha256 );
    EXPECT_EQ( actual.provenance.q2_commit, expected.provenance.q2_commit );
    EXPECT_EQ( actual.provenance.q2_tree, expected.provenance.q2_tree );
  }

  TEST( GyroAlignmentCliTest,
        EverySingleFieldV1MutantFailsBeforeFilesystemAccess )
  {
    using Mutator =
        std::function<void( GyroAlignmentProtocolDescriptor & descriptor )>;
    std::vector<Mutator> mutants;
    mutants.push_back( []( auto& d ) { d.protocol_id += "-mutant"; } );
    for ( std::size_t i = 0; i < 4U; ++i )
    {
      mutants.push_back( [ i ]( auto& d ) { d.inputs[ i ].basename += ".bad"; } );
      mutants.push_back(
          [ i ]( auto& d ) {
            toggleFirstHexDigit( d.inputs[ i ].expected_sha256 );
          } );
    }
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.protocol_identity.design_commit );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.protocol_identity.design_tree );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.protocol_identity.design_blob );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.protocol_identity.design_sha256 );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.analyzer_identity.source_commit );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.analyzer_identity.source_tree );
    } );
    mutants.push_back(
        []( auto& d ) { d.analyzer_identity.build_type += "-mutant"; } );
    mutants.push_back(
        []( auto& d ) { d.analyzer_identity.compiler += "-mutant"; } );
    mutants.push_back(
        []( auto& d ) { d.analyzer_identity.compiler_version += "-mutant"; } );
    mutants.push_back(
        []( auto& d ) { d.provenance.q1_source_run += "-mutant"; } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.provenance.q1_source_commit );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.provenance.q1_git_tree_object );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.provenance.q1_git_ls_tree_sha256 );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.provenance.q1_meta_sha256 );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.provenance.q1_config_hash );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.provenance.q1_input_manifest_v2_sha256 );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.provenance.q2_commit );
    } );
    mutants.push_back( []( auto& d ) {
      toggleFirstHexDigit( d.provenance.q2_tree );
    } );

    ScopedDirectory root{ "v1_mutants" };
    const auto      missing_q1 = root.path / "must-not-lstat";
    for ( std::size_t i = 0; i < mutants.size(); ++i )
    {
      auto descriptor = makeGyroAlignmentV1ProtocolDescriptor();
      mutants[ i ]( descriptor );
      const auto out = root.path / ( "must-not-create-" +
                                     std::to_string( i ) );
      expectInvalidDescriptorPreFilesystem( std::move( descriptor ),
                                            missing_q1, out );
    }
  }

  TEST( GyroAlignmentCliTest, DescriptorShapeAndProtocolMasqueradesAreRejected )
  {
    const auto                                   fixture  = firstZeroFixture();
    const auto                                   baseline = syntheticDescriptor( fixture );
    ScopedDirectory                              root{ "descriptor_shape" };
    const auto                                   missing_q1 = root.path / "missing-q1";
    std::vector<GyroAlignmentProtocolDescriptor> mutants;
    auto                                         fewer = syntheticDescriptor( fixture );
    fewer.inputs.pop_back();
    mutants.push_back( std::move( fewer ) );
    auto extra = syntheticDescriptor( fixture );
    extra.inputs.push_back( { "extra.csv", std::string( 64U, '8' ) } );
    mutants.push_back( std::move( extra ) );
    auto swapped = syntheticDescriptor( fixture );
    std::swap( swapped.inputs[ 0 ], swapped.inputs[ 1 ] );
    mutants.push_back( std::move( swapped ) );
    const auto expect_only_input_field_changed =
        [ & ]( const GyroAlignmentProtocolDescriptor& mutated,
               std::size_t target, bool basename_changed ) {
          ASSERT_EQ( mutated.inputs.size(), baseline.inputs.size() );
          for ( std::size_t i = 0; i < baseline.inputs.size(); ++i )
          {
            if ( i == target && basename_changed )
            {
              EXPECT_NE( mutated.inputs[ i ].basename,
                         baseline.inputs[ i ].basename );
            }
            else
            {
              EXPECT_EQ( mutated.inputs[ i ].basename,
                         baseline.inputs[ i ].basename );
            }
            if ( i == target && !basename_changed )
            {
              EXPECT_NE( mutated.inputs[ i ].expected_sha256,
                         baseline.inputs[ i ].expected_sha256 );
            }
            else
            {
              EXPECT_EQ( mutated.inputs[ i ].expected_sha256,
                         baseline.inputs[ i ].expected_sha256 );
            }
          }
        };
    for ( std::size_t i = 0; i < baseline.inputs.size(); ++i )
    {
      auto bad_basename = baseline;
      bad_basename.inputs[ i ].basename += ".bad";
      expect_only_input_field_changed( bad_basename, i, true );
      mutants.push_back( std::move( bad_basename ) );

      auto bad_hash_shape = baseline;
      bad_hash_shape.inputs[ i ].expected_sha256.pop_back();
      expect_only_input_field_changed( bad_hash_shape, i, false );
      mutants.push_back( std::move( bad_hash_shape ) );

      auto uppercase_hash = baseline;
      ASSERT_FALSE( uppercase_hash.inputs[ i ].expected_sha256.empty() );
      uppercase_hash.inputs[ i ].expected_sha256[ 0 ] = 'A';
      expect_only_input_field_changed( uppercase_hash, i, false );
      mutants.push_back( std::move( uppercase_hash ) );
    }
    auto unknown        = syntheticDescriptor( fixture );
    unknown.protocol_id = "PHAD-M4-Q3-UNKNOWN";
    mutants.push_back( std::move( unknown ) );
    auto synthetic_as_v1        = syntheticDescriptor( fixture );
    synthetic_as_v1.protocol_id = "PHAD-M4-Q3-GYRO-ALIGN-V1";
    mutants.push_back( std::move( synthetic_as_v1 ) );
    auto factory_as_synthetic        = makeGyroAlignmentV1ProtocolDescriptor();
    factory_as_synthetic.protocol_id = std::string{ kSyntheticProtocolId };
    mutants.push_back( std::move( factory_as_synthetic ) );

    for ( std::size_t i = 0; i < mutants.size(); ++i )
    {
      const auto out = root.path / ( "out-" + std::to_string( i ) );
      expectInvalidDescriptorPreFilesystem( std::move( mutants[ i ] ),
                                            missing_q1, out );
    }
  }

  TEST( GyroAlignmentCliTest,
        EverySyntheticIdentityFieldRejectsEmptyAndMalformedShapePreFilesystem )
  {
    using Mutator =
        std::function<void( GyroAlignmentProtocolDescriptor & descriptor )>;
    std::vector<Mutator> mutants;
    const auto           add_shape_mutants = [ & ]( auto field, std::string malformed ) {
      mutants.push_back( [ field ]( auto& descriptor ) {
        field( descriptor ).clear();
      } );
      mutants.push_back( [ field, malformed = std::move( malformed ) ](
                             auto& descriptor ) {
        field( descriptor ) = malformed;
      } );
    };
    const std::string bad_hex  = "not-lowercase-hex";
    const std::string bad_text = "contains\nnewline";
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.protocol_identity.design_commit;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.protocol_identity.design_tree;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.protocol_identity.design_blob;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.protocol_identity.design_sha256;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.analyzer_identity.source_commit;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.analyzer_identity.source_tree;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.analyzer_identity.build_type;
        },
        bad_text );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.analyzer_identity.compiler;
        },
        bad_text );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.analyzer_identity.compiler_version;
        },
        bad_text );
    add_shape_mutants(
        []( auto& d ) -> std::string& { return d.provenance.q1_source_run; },
        bad_text );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.provenance.q1_source_commit;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.provenance.q1_git_tree_object;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.provenance.q1_git_ls_tree_sha256;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.provenance.q1_meta_sha256;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& { return d.provenance.q1_config_hash; },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& {
          return d.provenance.q1_input_manifest_v2_sha256;
        },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& { return d.provenance.q2_commit; },
        bad_hex );
    add_shape_mutants(
        []( auto& d ) -> std::string& { return d.provenance.q2_tree; },
        bad_hex );

    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "synthetic_identity_shape" };
    const auto      missing_q1 = root.path / "must-not-lstat";
    for ( std::size_t i = 0; i < mutants.size(); ++i )
    {
      auto descriptor = syntheticDescriptor( fixture );
      mutants[ i ]( descriptor );
      const auto out = root.path / ( "must-not-create-" +
                                     std::to_string( i ) );
      expectInvalidDescriptorPreFilesystem( std::move( descriptor ),
                                            missing_q1, out );
    }
  }

  TEST( GyroAlignmentCliTest, FrozenFirstZeroHashesAndSizesAreExact )
  {
    const auto fixture    = firstZeroFixture();
    const auto descriptor = syntheticDescriptor( fixture );
    ASSERT_EQ( descriptor.inputs.size(), kFirstZeroHashes.size() );
    for ( std::size_t i = 0; i < kFirstZeroHashes.size(); ++i )
    {
      EXPECT_EQ( descriptor.inputs[ i ].expected_sha256,
                 kFirstZeroHashes[ i ] );
      EXPECT_EQ( descriptor.inputs[ i ].basename, kInputNames[ i ] );
    }
  }

  TEST( GyroAlignmentCliTest, RunnerPublishesAllThreeScientificVerdicts )
  {
    struct Case
    {
      std::string            name;
      FixtureBytes           fixture;
      GyroAlignmentStatus    status;
      GyroAlignmentFitStatus fit_status;
      int                    exit_code;
    };
    std::vector<Case> cases;
    cases.push_back( { "inconclusive", firstZeroFixture(),
                       GyroAlignmentStatus::kInconclusive,
                       GyroAlignmentFitStatus::kSupportInsufficient, 3 } );
    cases.push_back( { "hypothesis_fail",
                       stationaryFixture( "0", "0", "0",
                                          kZeroStationaryHashes ),
                       GyroAlignmentStatus::kHypothesisFail,
                       GyroAlignmentFitStatus::kSolved, 2 } );
    cases.push_back( { "pass",
                       stationaryFixture( "0.002", "-0.003", "0.001",
                                          kBiasStationaryHashes ),
                       GyroAlignmentStatus::kPass,
                       GyroAlignmentFitStatus::kSolved, 0 } );

    const auto zero_descriptor = syntheticDescriptor( cases[ 1 ].fixture );
    const auto bias_descriptor = syntheticDescriptor( cases[ 2 ].fixture );
    for ( std::size_t i = 0; i < kInputNames.size(); ++i )
    {
      EXPECT_EQ( zero_descriptor.inputs[ i ].expected_sha256,
                 kZeroStationaryHashes[ i ] );
      EXPECT_EQ( bias_descriptor.inputs[ i ].expected_sha256,
                 kBiasStationaryHashes[ i ] );
    }

    ScopedDirectory root{ "three_verdicts" };
    for ( auto& test_case : cases )
    {
      const auto case_root = root.path / test_case.name;
      std::filesystem::create_directory( case_root );
      const auto q1_dir  = materializeFixture( case_root, test_case.fixture );
      const auto out_dir = case_root / "out";
      const auto result  = runGyroAlignment(
          syntheticDescriptor( test_case.fixture ), q1_dir, out_dir );
      EXPECT_EQ( result.verdict.status, test_case.status );
      EXPECT_EQ( result.verdict.exit_code, test_case.exit_code );
      EXPECT_FALSE( result.error.has_value() );
      EXPECT_TRUE( result.analysis.has_value() );
      EXPECT_EQ( directoryEntries( out_dir ),
                 ( std::vector<std::string>{
                     "gyro_alignment.json", "gyro_alignment.manifest.json",
                     "gyro_alignment_blocks.csv" } ) );
      const auto summary = nlohmann::ordered_json::parse(
          readFile( out_dir / "gyro_alignment.json" ) );
      const auto manifest = nlohmann::ordered_json::parse(
          readFile( out_dir / "gyro_alignment.manifest.json" ) );
      ASSERT_TRUE( result.analysis.has_value() );
      expectFitArtifact( summary.at( "fit" ).at( "full" ),
                         test_case.fit_status,
                         result.analysis->full );
      expectFitArtifact( summary.at( "fit" ).at( "early" ),
                         test_case.fit_status,
                         result.analysis->early );
      expectFitArtifact( summary.at( "fit" ).at( "late" ),
                         test_case.fit_status,
                         result.analysis->late );
      EXPECT_EQ( summary.at( "verdict" ).at( "exit_code" ),
                 test_case.exit_code );
      EXPECT_EQ( manifest.at( "verdict" ).at( "exit_code" ),
                 test_case.exit_code );
    }
  }

  TEST( GyroAlignmentCliTest,
        RunnerSerializesEveryNonsolvedFitBranchFromPublicResult )
  {
    struct Case
    {
      std::string            name;
      FixtureBytes           fixture;
      GyroAlignmentFitStatus status;
    };
    std::vector<Case> cases;
    cases.push_back(
        { "rank_deficient",
          degenerateFitFixture( "314.15926535897933",
                                kRankDeficientHashes ),
          GyroAlignmentFitStatus::kRankDeficient } );
    cases.push_back(
        { "ill_conditioned",
          degenerateFitFixture( "314.15901535897933",
                                kIllConditionedHashes ),
          GyroAlignmentFitStatus::kIllConditioned } );
    cases.push_back( { "observability_failed", observabilityFixture(),
                       GyroAlignmentFitStatus::kObservabilityFailed } );

    ScopedDirectory root{ "fit_artifact_branches" };
    for ( auto& test_case : cases )
    {
      const auto case_root = root.path / test_case.name;
      std::filesystem::create_directory( case_root );
      const auto q1_dir  = materializeFixture( case_root, test_case.fixture );
      const auto out_dir = case_root / "out";
      const auto result  = runGyroAlignment(
          syntheticDescriptor( test_case.fixture ), q1_dir, out_dir );
      ASSERT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
      ASSERT_TRUE( result.analysis.has_value() );
      const auto summary = nlohmann::ordered_json::parse(
          readFile( out_dir / "gyro_alignment.json" ) );
      const std::array<const GyroAlignmentFit*, 3> public_fits{
          &result.analysis->full,
          &result.analysis->early,
          &result.analysis->late,
      };
      const std::array<std::string_view, 3> names{ "full", "early", "late" };
      for ( std::size_t i = 0; i < public_fits.size(); ++i )
      {
        EXPECT_EQ( public_fits[ i ]->status, test_case.status );
        expectFitArtifact(
            summary.at( "fit" ).at( std::string{ names[ i ] } ),
            test_case.status,
            *public_fits[ i ] );
      }
    }
  }

  TEST( GyroAlignmentCliTest, SummarySchemaOrderTypesAndNullabilityAreExact )
  {
    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "summary_schema" };
    const auto      q1_dir     = materializeFixture( root.path, fixture );
    const auto      out_dir    = root.path / "out";
    const auto      descriptor = syntheticDescriptor( fixture );
    const auto      result     = runGyroAlignment( descriptor, q1_dir, out_dir );
    ASSERT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    const std::string summary_bytes =
        readFile( out_dir / "gyro_alignment.json" );
    ASSERT_FALSE( summary_bytes.empty() );
    EXPECT_EQ( summary_bytes.back(), '\n' );
    EXPECT_NE( summary_bytes.find( "\n  \"protocol_id\"" ),
               std::string::npos );
    const auto summary = nlohmann::ordered_json::parse( summary_bytes );
    EXPECT_EQ( keys( summary ),
               ( std::vector<std::string>{
                   "schema_version", "protocol_id", "protocol_identity",
                   "analyzer_identity", "summary", "provenance", "inputs",
                   "ranges", "eligibility", "fit", "support", "gates",
                   "verdict" } ) );
    EXPECT_EQ( summary.at( "schema_version" ),
               "phad.gyro_alignment.v1" );
    EXPECT_EQ( summary.at( "protocol_id" ), descriptor.protocol_id );
    EXPECT_EQ( keys( summary.at( "protocol_identity" ) ),
               ( std::vector<std::string>{ "design_commit", "design_tree",
                                           "design_blob",
                                           "design_sha256" } ) );
    EXPECT_EQ( keys( summary.at( "analyzer_identity" ) ),
               ( std::vector<std::string>{ "source_commit", "source_tree",
                                           "build_type", "compiler",
                                           "compiler_version" } ) );
    expectProtocolIdentity( summary.at( "protocol_identity" ),
                            descriptor.protocol_identity );
    expectAnalyzerIdentity( summary.at( "analyzer_identity" ),
                            descriptor.analyzer_identity );
    EXPECT_EQ( keys( summary.at( "summary" ) ),
               ( std::vector<std::string>{
                   "claim", "offline_future_data", "physical_bias",
                   "online_initializer", "product_benefit", "bias_role",
                   "pass_authority" } ) );
    const auto& claim = summary.at( "summary" );
    EXPECT_EQ( claim.at( "claim" ),
               "frozen_visual_proxy_suffix_rotation_consistency" );
    EXPECT_TRUE( claim.at( "offline_future_data" ).get<bool>() );
    EXPECT_FALSE( claim.at( "physical_bias" ).get<bool>() );
    EXPECT_FALSE( claim.at( "online_initializer" ).get<bool>() );
    EXPECT_FALSE( claim.at( "product_benefit" ).get<bool>() );
    EXPECT_EQ( claim.at( "bias_role" ),
               "visual_posterior_aligned_nuisance" );
    EXPECT_EQ( claim.at( "pass_authority" ),
               "q4_controlled_factor_plan_only" );
    EXPECT_EQ( keys( summary.at( "provenance" ) ),
               ( std::vector<std::string>{
                   "q1_source_run", "q1_source_commit", "q1_git_tree_object",
                   "q1_git_ls_tree_sha256", "q1_meta_sha256", "q1_config_hash",
                   "q1_input_manifest_v2_sha256", "q2_commit", "q2_tree" } ) );
    expectProvenance( summary.at( "provenance" ), descriptor.provenance );
    ASSERT_EQ( summary.at( "inputs" ).size(), 4U );
    for ( std::size_t i = 0; i < kInputNames.size(); ++i )
    {
      const auto& item = summary.at( "inputs" ).at( i );
      EXPECT_EQ( keys( item ),
                 ( std::vector<std::string>{ "name", "size_bytes",
                                             "sha256" } ) );
      EXPECT_EQ( item.at( "name" ), kInputNames[ i ] );
      EXPECT_TRUE( item.at( "size_bytes" ).is_number_unsigned() );
      EXPECT_EQ( item.at( "sha256" ),
                 descriptor.inputs[ i ].expected_sha256 );
    }
    EXPECT_EQ( keys( summary.at( "ranges" ) ),
               ( std::vector<std::string>{ "t0_ns", "t_mid_ns",
                                           "t_split_ns",
                                           "validation_last_end_ns" } ) );
    EXPECT_TRUE(
        summary.at( "ranges" ).at( "validation_last_end_ns" ).is_null() );
    EXPECT_EQ( keys( summary.at( "fit" ) ),
               ( std::vector<std::string>{ "full", "early", "late" } ) );
    EXPECT_EQ( keys( summary.at( "eligibility" ) ),
               ( std::vector<std::string>{
                   "packet_total_nonfirst", "packet_structurally_eligible",
                   "packet_fit_eligible", "interval_excluded_counts",
                   "validation_slots_total", "validation_blocks_eligible",
                   "block_excluded_counts" } ) );
    EXPECT_EQ( keys( summary.at( "eligibility" )
                         .at( "interval_excluded_counts" ) ),
               ( std::vector<std::string>{
                   "gap", "empty_nonfirst", "diag_rejected_no_pose",
                   "segment_boundary", "near_pi" } ) );
    EXPECT_EQ( keys( summary.at( "eligibility" )
                         .at( "block_excluded_counts" ) ),
               ( std::vector<std::string>{ "incomplete_calendar_block",
                                           "near_pi" } ) );
    for ( const auto name : { "full", "early", "late" } )
    {
      const auto& fit = summary.at( "fit" ).at( name );
      EXPECT_EQ( keys( fit ),
                 std::vector<std::string>( kFitArtifactKeys.begin(),
                                           kFitArtifactKeys.end() ) );
      EXPECT_EQ( fit.at( "status" ), "support_insufficient" );
      EXPECT_TRUE( fit.at( "bias_radps" ).is_null() );
      EXPECT_TRUE( fit.at( "singular_values" ).is_null() );
      EXPECT_TRUE( fit.at( "rank" ).is_null() );
      EXPECT_TRUE( fit.at( "condition" ).is_null() );
      EXPECT_EQ( fit.at( "eligible_duration_ns" ), 0 );
      EXPECT_EQ( fit.at( "packet_count" ), 0U );
      EXPECT_TRUE( fit.at( "loss_zero_rad2" ).is_null() );
      EXPECT_TRUE( fit.at( "loss_fit_rad2" ).is_null() );
      EXPECT_TRUE( fit.at( "nonlinear_nonincrease" ).is_null() );
    }
    EXPECT_EQ( keys( summary.at( "support" ) ),
               ( std::vector<std::string>{
                   "full_duration_ns", "early_duration_ns", "late_duration_ns",
                   "validation_blocks", "sufficient" } ) );
    const auto& gates = summary.at( "gates" );
    EXPECT_EQ( keys( gates ),
               ( std::vector<std::string>{
                   "evaluated", "validation_loss_zero_rad2",
                   "validation_loss_fit_rad2", "blocks_improved",
                   "blocks_total", "axis_loss_zero_rad2",
                   "axis_loss_fit_rad2", "half_bias_max_abs_diff_radps",
                   "aggregate_reduction", "improved_fraction",
                   "axis_nonworsening", "half_stability", "all_passed" } ) );
    EXPECT_FALSE( gates.at( "evaluated" ).get<bool>() );
    for ( auto it = std::next( gates.begin() ); it != gates.end(); ++it )
    {
      EXPECT_TRUE( it.value().is_null() ) << it.key();
    }
    EXPECT_EQ( summary.at( "verdict" ).at( "status" ), "INCONCLUSIVE" );
    EXPECT_EQ( keys( summary.at( "verdict" ) ),
               ( std::vector<std::string>{ "status", "exit_code",
                                           "reason_codes", "detail" } ) );
    EXPECT_EQ( summary.at( "verdict" ).at( "exit_code" ), 3 );
    EXPECT_EQ( summary.at( "verdict" ).at( "reason_codes" ),
               nlohmann::ordered_json::array( { "FIT_SUPPORT" } ) );
  }

  TEST( GyroAlignmentCliTest, BlocksCsvUsesExactHeaderAndRoundTripDoubles )
  {
    const auto fixture =
        stationaryFixture( "0.002", "-0.003", "0.001",
                           kBiasStationaryHashes );
    ScopedDirectory root{ "blocks_schema" };
    const auto      q1_dir  = materializeFixture( root.path, fixture );
    const auto      out_dir = root.path / "out";
    const auto      result  = runGyroAlignment( syntheticDescriptor( fixture ),
                                                q1_dir, out_dir );
    ASSERT_EQ( result.verdict.status, GyroAlignmentStatus::kPass );
    const std::string csv =
        readFile( out_dir / "gyro_alignment_blocks.csv" );
    ASSERT_EQ( csv.substr( 0, kBlocksHeader.size() ), kBlocksHeader );
    std::istringstream lines{ csv };
    std::string        line;
    std::getline( lines, line );
    std::uint64_t expected_index = 0U;
    while ( std::getline( lines, line ) )
    {
      ASSERT_FALSE( line.empty() );
      std::vector<std::string> columns;
      std::istringstream       row{ line };
      std::string              column;
      while ( std::getline( row, column, ',' ) )
      {
        columns.push_back( column );
      }
      ASSERT_EQ( columns.size(), 15U );
      EXPECT_EQ( columns[ 0 ], "1" );
      EXPECT_EQ( std::stoull( columns[ 1 ] ), expected_index );
      EXPECT_EQ( columns[ 14 ], "1" );
      std::array<double, 8> parsed{};
      for ( std::size_t i = 6U; i <= 13U; ++i )
      {
        std::size_t  consumed = 0U;
        const double value    = std::stod( columns[ i ], &consumed );
        EXPECT_EQ( consumed, columns[ i ].size() );
        EXPECT_TRUE( std::isfinite( value ) );
        parsed[ i - 6U ] = value;
      }
      ASSERT_TRUE( result.analysis.has_value() );
      const auto& block = result.analysis->blocks.at( expected_index );
      EXPECT_DOUBLE_EQ( parsed[ 0 ], block.r_zero_rad.x() );
      EXPECT_DOUBLE_EQ( parsed[ 1 ], block.r_zero_rad.y() );
      EXPECT_DOUBLE_EQ( parsed[ 2 ], block.r_zero_rad.z() );
      EXPECT_DOUBLE_EQ( parsed[ 3 ], block.r_fit_rad.x() );
      EXPECT_DOUBLE_EQ( parsed[ 4 ], block.r_fit_rad.y() );
      EXPECT_DOUBLE_EQ( parsed[ 5 ], block.r_fit_rad.z() );
      EXPECT_DOUBLE_EQ( parsed[ 6 ], block.loss_zero_rad2 );
      EXPECT_DOUBLE_EQ( parsed[ 7 ], block.loss_fit_rad2 );
      ++expected_index;
    }
    EXPECT_EQ( expected_index, 60U );
  }

  TEST( GyroAlignmentCliTest,
        RejectedEndpointNeverCreatesCrossFrameBlockRow )
  {
    constexpr std::int64_t kRejectedNs     = 130'000'000'000LL;
    const auto             control_fixture = stationaryFixture(
        "0.002", "-0.003", "0.001", kMarginStationaryHashes, false, 162U );
    const auto rejected_fixture = stationaryFixture(
        "0.002", "-0.003", "0.001", kRejectedStationaryHashes, false, 162U,
        130U );
    ScopedDirectory root{ "rejected_endpoint" };
    const auto      control_root  = root.path / "control";
    const auto      rejected_root = root.path / "rejected";
    std::filesystem::create_directory( control_root );
    std::filesystem::create_directory( rejected_root );
    const auto control_q1 = materializeFixture( control_root, control_fixture );
    const auto rejected_q1 =
        materializeFixture( rejected_root, rejected_fixture );
    const auto control_out  = control_root / "out";
    const auto rejected_out = rejected_root / "out";
    const auto control      = runGyroAlignment(
        syntheticDescriptor( control_fixture ), control_q1, control_out );
    const auto rejected = runGyroAlignment(
        syntheticDescriptor( rejected_fixture ), rejected_q1, rejected_out );
    ASSERT_EQ( control.verdict.status, GyroAlignmentStatus::kPass );
    ASSERT_EQ( rejected.verdict.status, control.verdict.status );

    const auto control_json = nlohmann::ordered_json::parse(
        readFile( control_out / "gyro_alignment.json" ) );
    const auto rejected_json = nlohmann::ordered_json::parse(
        readFile( rejected_out / "gyro_alignment.json" ) );
    const auto& control_elig  = control_json.at( "eligibility" );
    const auto& rejected_elig = rejected_json.at( "eligibility" );
    EXPECT_EQ( rejected_elig.at( "packet_total_nonfirst" ),
               control_elig.at( "packet_total_nonfirst" ) );
    EXPECT_EQ( rejected_elig.at( "packet_structurally_eligible" ).get<std::uint64_t>() + 2U,
               control_elig.at( "packet_structurally_eligible" ).get<std::uint64_t>() );
    EXPECT_EQ( rejected_elig.at( "packet_fit_eligible" ).get<std::uint64_t>() + 2U,
               control_elig.at( "packet_fit_eligible" ).get<std::uint64_t>() );
    const auto& control_counts =
        control_elig.at( "interval_excluded_counts" );
    const auto& rejected_counts =
        rejected_elig.at( "interval_excluded_counts" );
    EXPECT_EQ( rejected_counts.at( "diag_rejected_no_pose" ).get<std::uint64_t>(),
               control_counts.at( "diag_rejected_no_pose" ).get<std::uint64_t>() + 2U );
    for ( const auto reason : { "gap", "empty_nonfirst", "segment_boundary",
                                "near_pi" } )
    {
      EXPECT_EQ( rejected_counts.at( reason ), control_counts.at( reason ) );
    }
    const auto total = rejected_elig.at( "packet_total_nonfirst" ).get<std::uint64_t>();
    const auto structural =
        rejected_elig.at( "packet_structurally_eligible" ).get<std::uint64_t>();
    EXPECT_EQ( total,
               rejected_counts.at( "gap" ).get<std::uint64_t>() +
                   rejected_counts.at( "empty_nonfirst" ).get<std::uint64_t>() +
                   rejected_counts.at( "diag_rejected_no_pose" ).get<std::uint64_t>() +
                   rejected_counts.at( "segment_boundary" ).get<std::uint64_t>() +
                   structural );
    EXPECT_EQ( structural,
               rejected_elig.at( "packet_fit_eligible" ).get<std::uint64_t>() +
                   rejected_counts.at( "near_pi" ).get<std::uint64_t>() );
    EXPECT_TRUE(
        control_json.at( "support" ).at( "sufficient" ).get<bool>() );
    EXPECT_TRUE(
        rejected_json.at( "support" ).at( "sufficient" ).get<bool>() );
    EXPECT_EQ( rejected_json.at( "verdict" ).at( "status" ),
               control_json.at( "verdict" ).at( "status" ) );
    EXPECT_EQ( rejected_elig.at( "validation_blocks_eligible" ).get<std::uint64_t>(),
               60U );

    std::istringstream rows{
        readFile( rejected_out / "gyro_alignment_blocks.csv" ) };
    std::string row;
    ASSERT_TRUE( static_cast<bool>( std::getline( rows, row ) ) );
    EXPECT_EQ( row + "\n", kBlocksHeader );
    std::size_t row_count = 0U;
    while ( std::getline( rows, row ) )
    {
      std::istringstream       columns{ row };
      std::vector<std::string> values;
      std::string              value;
      while ( std::getline( columns, value, ',' ) )
      {
        values.push_back( value );
      }
      ASSERT_EQ( values.size(), 15U );
      const auto start_ns = std::stoll( values[ 2 ] );
      const auto end_ns   = std::stoll( values[ 3 ] );
      EXPECT_FALSE( start_ns <= kRejectedNs && kRejectedNs <= end_ns );
      ++row_count;
    }
    EXPECT_EQ( row_count, 60U );
  }

  TEST( GyroAlignmentCliTest, ValidPacketCannotCrossRejectedMiddleEndpoint )
  {
    auto                       fixture = stationaryFixture( "0.002", "-0.003", "0.001",
                                                            kRejectedStationaryHashes, false, 162U,
                                                            130U );
    constexpr std::string_view kPacketBefore =
        "131,130000000000,131000000000,0,0,2,1000000000,1000000000,valid\n";
    constexpr std::string_view kPacketAfter =
        "131,129000000000,131000000000,0,0,2,2000000000,2000000000,valid\n";
    constexpr std::string_view kSampleBefore =
        "131,0,130000000000,0.002,-0.003,0.001\n";
    constexpr std::string_view kSampleAfter =
        "131,0,129000000000,0.002,-0.003,0.001\n";
    const auto packet_position = fixture.files[ 2 ].find( kPacketBefore );
    const auto sample_position = fixture.files[ 3 ].find( kSampleBefore );
    ASSERT_NE( packet_position, std::string::npos );
    ASSERT_NE( sample_position, std::string::npos );
    ASSERT_EQ( fixture.files[ 2 ].find(
                   kPacketBefore, packet_position + kPacketBefore.size() ),
               std::string::npos );
    ASSERT_EQ( fixture.files[ 3 ].find(
                   kSampleBefore, sample_position + kSampleBefore.size() ),
               std::string::npos );
    fixture.files[ 2 ].replace( packet_position, kPacketBefore.size(),
                                kPacketAfter.data(), kPacketAfter.size() );
    fixture.files[ 3 ].replace( sample_position, kSampleBefore.size(),
                                kSampleAfter.data(), kSampleAfter.size() );
    fixture.hashes = kCrossedRejectedStationaryHashes;
    ScopedDirectory root{ "crossed_rejected_endpoint" };
    const auto      q1_dir  = materializeFixture( root.path, fixture );
    const auto      out_dir = root.path / "out";

    const auto result = runGyroAlignment( syntheticDescriptor( fixture ),
                                          q1_dir, out_dir );
    expectHard( result, GyroAlignmentErrorCode::kJoinMismatch );
    EXPECT_FALSE( std::filesystem::exists( out_dir ) );
  }

  TEST( GyroAlignmentCliTest, ManifestIsLastAndBindsExactInputOutputBytes )
  {
    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "manifest" };
    const auto      q1_dir     = materializeFixture( root.path, fixture );
    const auto      out_dir    = root.path / "out";
    const auto      descriptor = syntheticDescriptor( fixture );
    const auto      result     = runGyroAlignment( descriptor, q1_dir, out_dir );
    ASSERT_EQ( result.verdict.status, GyroAlignmentStatus::kInconclusive );
    const auto manifest = nlohmann::ordered_json::parse(
        readFile( out_dir / "gyro_alignment.manifest.json" ) );
    EXPECT_EQ( keys( manifest ),
               ( std::vector<std::string>{
                   "schema_version", "protocol_id", "protocol_identity",
                   "verdict", "science_outputs", "inputs" } ) );
    EXPECT_EQ( manifest.at( "schema_version" ),
               "phad.gyro_alignment.manifest.v1" );
    EXPECT_EQ( manifest.at( "protocol_id" ), descriptor.protocol_id );
    EXPECT_EQ( keys( manifest.at( "protocol_identity" ) ),
               ( std::vector<std::string>{ "design_commit", "design_tree",
                                           "design_blob",
                                           "design_sha256" } ) );
    expectProtocolIdentity( manifest.at( "protocol_identity" ),
                            descriptor.protocol_identity );
    EXPECT_FALSE( manifest.contains( "analyzer_identity" ) );
    EXPECT_FALSE( manifest.contains( "provenance" ) );
    EXPECT_EQ( keys( manifest.at( "verdict" ) ),
               ( std::vector<std::string>{ "status", "exit_code" } ) );
    ASSERT_EQ( manifest.at( "science_outputs" ).size(), 2U );
    ASSERT_EQ( manifest.at( "inputs" ).size(), 4U );
    constexpr std::array<std::string_view, 2> kOutputNames{
        "gyro_alignment.json", "gyro_alignment_blocks.csv" };
    for ( std::size_t i = 0; i < kOutputNames.size(); ++i )
    {
      const auto  path = out_dir / kOutputNames[ i ];
      const auto& item = manifest.at( "science_outputs" ).at( i );
      EXPECT_EQ( item.at( "name" ), kOutputNames[ i ] );
      EXPECT_EQ( item.at( "size_bytes" ),
                 std::filesystem::file_size( path ) );
      EXPECT_EQ( item.at( "sha256" ), sha256File( path ) );
    }
    for ( std::size_t i = 0; i < kInputNames.size(); ++i )
    {
      const auto  path = q1_dir / kInputNames[ i ];
      const auto& item = manifest.at( "inputs" ).at( i );
      EXPECT_EQ( item.at( "name" ), kInputNames[ i ] );
      EXPECT_EQ( item.at( "size_bytes" ), fixture.files[ i ].size() );
      EXPECT_EQ( item.at( "sha256" ), sha256File( path ) );
      EXPECT_EQ( item.at( "sha256" ),
                 descriptor.inputs[ i ].expected_sha256 );
    }
    EXPECT_FALSE( qualificationConsumerAccepts( manifest ) );
    for ( const auto& entry : directoryEntries( out_dir ) )
    {
      EXPECT_EQ( entry.find( ".tmp" ), std::string::npos );
    }
  }

  TEST( GyroAlignmentCliTest,
        SuccessfulPublishRenamesManifestStrictlyLast )
  {
#if defined( __linux__ ) && defined( __x86_64__ )
    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "publish_trace" };
    const auto      q1_dir     = materializeFixture( root.path, fixture );
    const auto      out_dir    = root.path / "out";
    const auto      descriptor = syntheticDescriptor( fixture );
    const auto      trace      = traceChildSyscalls( [ & ] {
      const auto result = runGyroAlignment( descriptor, q1_dir, out_dir );
      return result.verdict.status == GyroAlignmentStatus::kInconclusive
                           ? 0
                           : 1;
    } );
    ASSERT_EQ( trace.child_exit, 0 );
    EXPECT_TRUE( validPublishTrace( trace, out_dir ) );
    EXPECT_EQ( finalRenames( trace, out_dir ),
               ( std::vector<std::string>{
                   ( out_dir / "gyro_alignment.json" ).string(),
                   ( out_dir / "gyro_alignment_blocks.csv" ).string(),
                   ( out_dir / "gyro_alignment.manifest.json" ).string(),
               } ) );
#else
    FAIL() << "Q3 publish syscall oracle requires Linux x86_64";
#endif
  }

  TEST( GyroAlignmentCliTest,
        TraceOracleRejectsEarlyManifestAndIncompleteRollbackMutants )
  {
#if defined( __linux__ ) && defined( __x86_64__ )
    ScopedDirectory root{ "publish_mutants" };
    const auto      early_out = root.path / "early";
    const auto      early     = traceChildSyscalls( [ & ] {
      std::filesystem::create_directory( early_out );
      const std::array<std::string_view, 3> names{
          "gyro_alignment.manifest.json",
          "gyro_alignment.json",
          "gyro_alignment_blocks.csv",
      };
      for ( const auto name : names )
      {
        const auto temp = early_out / ( std::string{ name } + ".tmp" );
        std::ofstream{ temp, std::ios::binary } << "mutant\n";
        std::filesystem::rename( temp, early_out / std::string{ name } );
      }
      return 0;
    } );
    ASSERT_EQ( early.child_exit, 0 );
    EXPECT_FALSE( validPublishTrace( early, early_out ) );

    const auto rollback_out = root.path / "rollback";
    const auto rollback     = traceChildSyscalls( [ & ] {
      std::filesystem::create_directory( rollback_out );
      const auto temp = rollback_out / "gyro_alignment.json.tmp";
      std::ofstream{ temp, std::ios::binary } << "mutant\n";
      std::filesystem::rename( temp,
                                   rollback_out / "gyro_alignment.json" );
      return 1;
    } );
    ASSERT_EQ( rollback.child_exit, 1 );
    EXPECT_FALSE( validRollbackTrace( rollback, rollback_out ) );
    EXPECT_FALSE( std::filesystem::exists(
        rollback_out / "gyro_alignment.manifest.json" ) );
#else
    FAIL() << "Q3 publish mutant oracle requires Linux x86_64";
#endif
  }

  TEST( GyroAlignmentCliTest, ExistingOutputWinsWithoutOverwrite )
  {
    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "output_exists" };
    const auto      q1_dir  = materializeFixture( root.path, fixture );
    const auto      out_dir = root.path / "out";
    std::filesystem::create_directory( out_dir );
    writeFile( out_dir / "owned.txt", "keep\n" );

    const auto result = runGyroAlignment( syntheticDescriptor( fixture ),
                                          q1_dir, out_dir );
    expectHard( result, GyroAlignmentErrorCode::kOutputExists );
    EXPECT_EQ( readFile( out_dir / "owned.txt" ), "keep\n" );
    EXPECT_EQ( directoryEntries( out_dir ),
               ( std::vector<std::string>{ "owned.txt" } ) );
  }

  TEST( GyroAlignmentCliTest, OutputParentConflictIsTypedAndLeavesNoManifest )
  {
    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "output_io" };
    const auto      q1_dir      = materializeFixture( root.path, fixture );
    const auto      parent_file = root.path / "not-a-directory";
    writeFile( parent_file, "occupied\n" );
    const auto out_dir = parent_file / "out";

    const auto result = runGyroAlignment( syntheticDescriptor( fixture ),
                                          q1_dir, out_dir );
    expectHard( result, GyroAlignmentErrorCode::kOutputIo );
    EXPECT_FALSE( std::filesystem::exists(
        out_dir / "gyro_alignment.manifest.json" ) );
    EXPECT_EQ( readFile( parent_file ), "occupied\n" );
  }

  TEST( GyroAlignmentCliTest,
        ProductionHardFailureHasCleanRollbackTraceAndNoManifest )
  {
#if defined( __linux__ ) && defined( __x86_64__ )
    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "hard_rollback" };
    const auto      q1_dir      = materializeFixture( root.path, fixture );
    const auto      parent_file = root.path / "not-a-directory";
    writeFile( parent_file, "occupied\n" );
    const auto out_dir    = parent_file / "out";
    const auto descriptor = syntheticDescriptor( fixture );
    const auto trace      = traceChildSyscalls( [ & ] {
      const auto result = runGyroAlignment( descriptor, q1_dir, out_dir );
      return result.verdict.status == GyroAlignmentStatus::kHardError &&
                     result.error == GyroAlignmentErrorCode::kOutputIo
                      ? 0
                      : 1;
    } );
    ASSERT_EQ( trace.child_exit, 0 );
    EXPECT_TRUE( validRollbackTrace( trace, out_dir ) );
    EXPECT_FALSE( std::filesystem::exists(
        out_dir / "gyro_alignment.manifest.json" ) );
#else
    FAIL() << "Q3 rollback syscall oracle requires Linux x86_64";
#endif
  }

  TEST( GyroAlignmentCliTest,
        PartialProductionPublishRollsBackAfterInjectedRenameEio )
  {
#if defined( __linux__ ) && defined( __x86_64__ )
    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "partial_publish_rollback" };
    const auto      q1_dir     = materializeFixture( root.path, fixture );
    const auto      out_dir    = root.path / "out";
    const auto      descriptor = syntheticDescriptor( fixture );
    const auto      failed_path =
        ( out_dir / "gyro_alignment_blocks.csv" ).string();
    const auto trace = traceChildSyscalls(
        [ & ] {
          const auto result =
              runGyroAlignment( descriptor, q1_dir, out_dir );
          return result.verdict.status == GyroAlignmentStatus::kHardError &&
                         result.verdict.exit_code == 1 &&
                         result.error ==
                             GyroAlignmentErrorCode::kOutputPublish &&
                         !result.analysis.has_value()
                     ? 0
                     : 1;
        },
        TraceFault{ failed_path } );
    ASSERT_EQ( trace.child_exit, 0 );
    ASSERT_TRUE( trace.fault_injected );
    EXPECT_NE( std::find( trace.rename_attempts.begin(),
                          trace.rename_attempts.end(), failed_path ),
               trace.rename_attempts.end() );
    EXPECT_EQ( finalRenames( trace, out_dir ),
               ( std::vector<std::string>{
                   ( out_dir / "gyro_alignment.json" ).string() } ) );
    EXPECT_NE( std::find( trace.unlinked_paths.begin(),
                          trace.unlinked_paths.end(),
                          ( out_dir / "gyro_alignment.json" ).string() ),
               trace.unlinked_paths.end() );
    EXPECT_TRUE( validRollbackTrace( trace, out_dir ) );
    EXPECT_EQ( std::find( trace.rename_attempts.begin(),
                          trace.rename_attempts.end(),
                          ( out_dir / "gyro_alignment.manifest.json" ).string() ),
               trace.rename_attempts.end() );
    EXPECT_FALSE( std::filesystem::exists(
        out_dir / "gyro_alignment.manifest.json" ) );
    EXPECT_TRUE( directoryEntries( out_dir ).empty() );
#else
    FAIL() << "Q3 partial publish fault oracle requires Linux x86_64";
#endif
  }

  TEST( GyroAlignmentCliTest, HashMismatchPrecedesMalformedGrammar )
  {
    auto fixture    = firstZeroFixture();
    auto descriptor = syntheticDescriptor( fixture );
    fixture.files[ 1 ] += "malformed,row\n";
    ScopedDirectory root{ "hash_before_parse" };
    const auto      q1_dir  = materializeFixture( root.path, fixture );
    const auto      out_dir = root.path / "out";
    const auto      result =
        runGyroAlignment( std::move( descriptor ), q1_dir, out_dir );
    expectHard( result, GyroAlignmentErrorCode::kInputHashMismatch );
    EXPECT_FALSE( std::filesystem::exists( out_dir ) );
  }

  TEST( GyroAlignmentCliTest, EachImmutableInputBufferIsOpenedExactlyOnce )
  {
#if defined( __linux__ ) && defined( __x86_64__ )
    const auto                                               fixture = firstZeroFixture();
    ScopedDirectory                                          root{ "one_open" };
    const auto                                               q1_dir     = materializeFixture( root.path, fixture );
    const auto                                               out_dir    = root.path / "out";
    const auto                                               descriptor = syntheticDescriptor( fixture );
    std::array<std::pair<std::uintmax_t, std::uintmax_t>, 4> identities{};
    for ( std::size_t i = 0; i < kInputNames.size(); ++i )
    {
      identities[ i ] =
          fileIdentity( q1_dir / std::string{ kInputNames[ i ] } );
    }
    const auto trace = traceChildSyscalls( [ & ] {
      const auto result = runGyroAlignment( descriptor, q1_dir, out_dir );
      return result.verdict.status == GyroAlignmentStatus::kInconclusive
                 ? 0
                 : 1;
    } );
    ASSERT_EQ( trace.child_exit, 0 );
    for ( std::size_t i = 0; i < identities.size(); ++i )
    {
      const auto count = std::count_if(
          trace.opened_files.begin(), trace.opened_files.end(),
          [ & ]( const auto& opened ) {
            return opened.device == identities[ i ].first &&
                   opened.inode == identities[ i ].second;
          } );
      EXPECT_EQ( count, 1 ) << kInputNames[ i ];
    }
#else
    FAIL() << "Q3 one-read syscall oracle requires Linux x86_64";
#endif
  }

  TEST( GyroAlignmentCliTest, StrictQ3TumRejectsGenericCommentsAndBlankRows )
  {
    struct Case
    {
      std::string      bytes;
      std::string_view hash;
    };
    const std::array<Case, 2> cases{
        Case{ "# generic TUM comment\n0.000000000 0 0 0 0 0 0 1\n",
              "5e4a52ab1149a46baaa78a07591260f8ad52258781730145ea695c4a30d01000" },
        Case{ "0.000000000 0 0 0 0 0 0 1\n\n",
              "17410bbf4489911976d2a8aca91fc7c6981d4dd1001204853c7261ab83cab86d" },
    };
    for ( const auto& test_case : cases )
    {
      const auto generic =
          phad::eval::readTumBytes( test_case.bytes, "memory.tum" );
      EXPECT_TRUE( generic.hasValue() );
      auto fixture        = firstZeroFixture();
      fixture.files[ 0 ]  = test_case.bytes;
      fixture.hashes[ 0 ] = test_case.hash;
      ScopedDirectory root{ "strict_tum" };
      const auto      q1_dir = materializeFixture( root.path, fixture );
      const auto      result = runGyroAlignment( syntheticDescriptor( fixture ),
                                                 q1_dir, root.path / "out" );
      expectHard( result, GyroAlignmentErrorCode::kInputGrammar );
    }
  }

  TEST( GyroAlignmentCliTest, StrictGrammarAndSchemaMutantsAreTyped )
  {
    struct Mutation
    {
      std::string                          name;
      std::function<void( FixtureBytes& )> apply;
      std::size_t                          hash_index;
      std::string_view                     frozen_hash;
      GyroAlignmentErrorCode               expected;
    };
    const std::vector<Mutation> mutations{
        { "tum_extra_field", []( auto& f ) { f.files[ 0 ] =
                                                 "0.000000000 0 0 0 0 0 0 1 9\n"; },
          0U, "3f5bbf22b4620116f268ce7b632c78bfe9e013a88a2136425a5e507b5b03b48c",
          GyroAlignmentErrorCode::kInputSchema },
        { "tum_noncanonical_time", []( auto& f ) { f.files[ 0 ] =
                                                       "+0 0 0 0 0 0 0 1\n"; },
          0U, "6858b3fb9a9d7c196f4c9d3c040d88aa7427e8c330eb62028e44f7094c5d6589",
          GyroAlignmentErrorCode::kInputGrammar },
        { "tum_nan", []( auto& f ) { f.files[ 0 ] =
                                         "0.000000000 nan 0 0 0 0 0 1\n"; },
          0U, "5cf858a296c469b940142df4f08d89a92f5ecdd9f4a27ac7e564a7e69e865c4c",
          GyroAlignmentErrorCode::kNonfiniteInput },
        { "csv_quoted", []( auto& f ) { f.files[ 2 ].replace(
                                            f.files[ 2 ].find( "first_zero" ), 10U, "\"first_zero\"" ); },
          2U, "3ab8c02f8adbd0e44766bc59e355862db1b8d5848b429279f5fdda4f12ddf271",
          GyroAlignmentErrorCode::kInputGrammar },
        { "csv_unknown_enum", []( auto& f ) { f.files[ 2 ].replace(
                                                  f.files[ 2 ].find( "first_zero" ), 10U, "FIRST_ZERO" ); },
          2U, "ad1104e8042697490dc906f090a2f87b1b81d24b3d9529f57d869cd83d86c103",
          GyroAlignmentErrorCode::kUnknownEnum },
        { "csv_bad_bool", []( auto& f ) { f.files[ 2 ].replace(
                                              f.files[ 2 ].find( ",0,0,0,0,first_zero" ), 2U,
                                              ",true" ); },
          2U, "8626c2ead0ccf0433ee9a0c7a90e900daac26246ec70b1559dca37a4164ddca7",
          GyroAlignmentErrorCode::kInputGrammar },
        { "csv_blank", []( auto& f ) { f.files[ 3 ] += "\n"; },
          3U, "111797e134e3bd5cd3f38a0410f6da67d1d836f581a2b190a508fd60761c110a",
          GyroAlignmentErrorCode::kInputGrammar },
        { "csv_cr_only", []( auto& f ) { f.files[ 3 ].back() = '\r'; },
          3U, "0fb813881ed320e35827739d5e004395a19373f0e183b1f3093d03a3d40a9d31",
          GyroAlignmentErrorCode::kInputGrammar },
        { "wrong_header", []( auto& f ) { f.files[ 1 ].replace(
                                              0U, std::string{ "timestamp_ns" }.size(), "timestamp" ); },
          1U, "5ed7e723f1d0f4b860fc135414197c4f531933fdb042c02c727fd84cb6ae78d3",
          GyroAlignmentErrorCode::kInputSchema },
        { "extra_column", []( auto& f ) { f.files[ 1 ].insert(
                                              f.files[ 1 ].size() - 1U, ",extra" ); },
          1U, "060eec6c2fef500b75c4555d1e4341dc11664f838009928a8b74c63673dfb531",
          GyroAlignmentErrorCode::kInputSchema },
    };

    ScopedDirectory root{ "grammar_mutants" };
    for ( const auto& mutation : mutations )
    {
      auto fixture = firstZeroFixture();
      mutation.apply( fixture );
      fixture.hashes[ mutation.hash_index ] = mutation.frozen_hash;
      const auto case_root                  = root.path / mutation.name;
      std::filesystem::create_directory( case_root );
      const auto q1_dir  = materializeFixture( case_root, fixture );
      const auto out_dir = case_root / "out";
      const auto result  = runGyroAlignment( syntheticDescriptor( fixture ),
                                             q1_dir, out_dir );
      expectHard( result, mutation.expected );
      EXPECT_FALSE( std::filesystem::exists( out_dir ) );
    }
  }

  TEST( GyroAlignmentCliTest, FileIdentityRejectsMissingDirectoryAndSymlink )
  {
    const auto      fixture = firstZeroFixture();
    ScopedDirectory root{ "file_identity" };
    const auto      q1_dir = materializeFixture( root.path, fixture );
    std::filesystem::remove( q1_dir / "diag.csv" );
    auto result = runGyroAlignment( syntheticDescriptor( fixture ), q1_dir,
                                    root.path / "missing-out" );
    expectHard( result, GyroAlignmentErrorCode::kInputNotFound );

    writeFile( q1_dir / "diag.real", fixture.files[ 1 ] );
    std::filesystem::create_symlink( "diag.real", q1_dir / "diag.csv" );
    result = runGyroAlignment( syntheticDescriptor( fixture ), q1_dir,
                               root.path / "symlink-out" );
    expectHard( result, GyroAlignmentErrorCode::kInputNotRegularFile );

    std::filesystem::remove( q1_dir / "diag.csv" );
    std::filesystem::create_directory( q1_dir / "diag.csv" );
    result = runGyroAlignment( syntheticDescriptor( fixture ), q1_dir,
                               root.path / "directory-out" );
    expectHard( result, GyroAlignmentErrorCode::kInputNotRegularFile );
  }

  TEST( GyroAlignmentCliTest, JoinCountDurationAndEndpointMutantsAreTyped )
  {
    struct Mutation
    {
      std::string                          name;
      std::function<void( FixtureBytes& )> apply;
      std::size_t                          hash_index;
      std::string_view                     frozen_hash;
      GyroAlignmentErrorCode               expected;
    };
    const std::vector<Mutation> mutations{
        { "orphan_pose", []( auto& f ) { f.files[ 0 ] +=
                                             "1.000000000 0 0 0 0 0 0 1\n"; },
          0U, "7b3a1906735b3f08e7b7fad7cbdf3d23f65bb04bb344180797b3dda04c2be3df",
          GyroAlignmentErrorCode::kJoinMismatch },
        { "orphan_diag", []( auto& f ) { f.files[ 1 ] +=
                                             "1000000000,ok,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0\n"; },
          1U, "3422aca5f781678a1bd12a14b7727ad38892f24cd6146eb6a5db9cbf874fd5fd",
          GyroAlignmentErrorCode::kJoinMismatch },
        { "packet_index", []( auto& f ) { f.files[ 2 ].replace(
                                              f.files[ 2 ].find( "0,0,0" ), 1U, "1" ); },
          2U, "76e6d7a086fb8b30d12908ecc6ebe5309d4e2d6752938ad13588b8125d65c955",
          GyroAlignmentErrorCode::kIndexOrder },
        { "first_zero_count", []( auto& f ) { f.files[ 2 ].replace(
                                                  f.files[ 2 ].find( ",0,0,0,first_zero" ), 2U, ",1" ); },
          2U, "a15b1664de987d1639d5c2903c56dd634fe18b18ba526e9a530152430be2e576",
          GyroAlignmentErrorCode::kCountMismatch },
        { "first_zero_duration", []( auto& f ) { f.files[ 2 ].replace(
                                                     f.files[ 2 ].find( ",0,first_zero" ), 2U, ",1" ); },
          2U, "cf87ac2a75ae7b2815c8d7e93d8d5412ba38cc646101c48260840002c02dc130",
          GyroAlignmentErrorCode::kDurationMismatch },
    };
    ScopedDirectory root{ "join_mutants" };
    for ( const auto& mutation : mutations )
    {
      auto fixture = firstZeroFixture();
      mutation.apply( fixture );
      fixture.hashes[ mutation.hash_index ] = mutation.frozen_hash;
      const auto case_root                  = root.path / mutation.name;
      std::filesystem::create_directory( case_root );
      const auto q1_dir = materializeFixture( case_root, fixture );
      const auto result = runGyroAlignment( syntheticDescriptor( fixture ),
                                            q1_dir, case_root / "out" );
      expectHard( result, mutation.expected );
    }
  }

  TEST( GyroAlignmentCliTest, ScientificArtifactHasNoGroundtruthOutcomeLeakage )
  {
    const auto fixture =
        stationaryFixture( "0.002", "-0.003", "0.001",
                           kMixedBiasStationaryHashes, true );
    ScopedDirectory root{ "no_gt" };
    const auto      q1_dir  = materializeFixture( root.path, fixture );
    const auto      out_dir = root.path / "out";
    const auto      result  = runGyroAlignment( syntheticDescriptor( fixture ),
                                                q1_dir, out_dir );
    ASSERT_NE( result.verdict.status, GyroAlignmentStatus::kHardError );
    const auto summary = nlohmann::ordered_json::parse(
        readFile( out_dir / "gyro_alignment.json" ) );
    const auto manifest = nlohmann::ordered_json::parse(
        readFile( out_dir / "gyro_alignment.manifest.json" ) );
    expectNoOutcomeKeys( summary );
    expectNoOutcomeKeys( manifest );

    std::istringstream blocks{
        readFile( out_dir / "gyro_alignment_blocks.csv" ) };
    std::string header;
    ASSERT_TRUE( static_cast<bool>( std::getline( blocks, header ) ) );
    std::istringstream header_stream{ header };
    std::string        column;
    while ( std::getline( header_stream, column, ',' ) )
    {
      EXPECT_NE( column, "groundtruth" );
      EXPECT_NE( column, "ground_truth" );
      EXPECT_NE( column, "ate_rmse" );
      EXPECT_NE( column, "dataset_locator" );
      EXPECT_NE( column.rfind( "rpe_", 0U ), 0U );
    }
  }

  TEST( GyroAlignmentCliTest, CliGrammarReturnsUsageWithoutCreatingOutput )
  {
    ASSERT_FALSE( std::string_view{ PHAD_GYRO_ALIGN_PATH }.empty() );
    ScopedDirectory                root{ "cli_usage" };
    const std::vector<std::string> invalid_args{
        "", "only-q1", "q1 --out", "q1 --out a extra", "--out a q1",
        "q1 --output a", "q1 --out a --protocol synthetic" };
    for ( std::size_t i = 0; i < invalid_args.size(); ++i )
    {
      const auto stdout_path = root.path / ( "stdout-" + std::to_string( i ) );
      const auto stderr_path = root.path / ( "stderr-" + std::to_string( i ) );
      const int  exit_code =
          runCommand( invalid_args[ i ], stdout_path, stderr_path );
      EXPECT_EQ( exit_code, 64 );
      const std::string stderr_text = readFile( stderr_path );
      EXPECT_NE( stderr_text.find(
                     "phad_gyro_align <q1-run-dir> --out <new-output-dir>" ),
                 std::string::npos );
      EXPECT_EQ( stderr_text.find( "descriptor" ), std::string::npos );
      EXPECT_EQ( stderr_text.find( "environment" ), std::string::npos );
    }
    EXPECT_EQ( directoryEntries( root.path ).size(),
               invalid_args.size() * 2U );
  }

  TEST( GyroAlignmentCliTest, CliAcceptedGrammarPropagatesHardExit )
  {
    ScopedDirectory root{ "cli_hard" };
    const auto      stdout_path = root.path / "stdout";
    const auto      stderr_path = root.path / "stderr";
    const auto      out_dir     = root.path / "out";
    const int       exit_code   = runCommand(
        "\"" + ( root.path / "missing-q1" ).string() + "\" --out \"" +
            out_dir.string() + "\"",
        stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 1 );
    EXPECT_FALSE( std::filesystem::exists( out_dir ) );
    EXPECT_NE( readFile( stderr_path ).find( "INPUT_NOT_FOUND" ),
               std::string::npos );
  }

  TEST( GyroAlignmentCliTest, CliIgnoresProtocolEnvironmentOverride )
  {
#ifdef __linux__
    ScopedDirectory root{ "cli_no_env_override" };
    const auto      stdout_path = root.path / "stdout";
    const auto      stderr_path = root.path / "stderr";
    const auto      out_dir     = root.path / "out";
    ASSERT_EQ( ::setenv( "PHAD_Q3_PROTOCOL_ID", kSyntheticProtocolId.data(),
                         1 ),
               0 );
    const int exit_code = runCommand(
        "\"" + ( root.path / "missing-q1" ).string() + "\" --out \"" +
            out_dir.string() + "\"",
        stdout_path, stderr_path );
    EXPECT_EQ( ::unsetenv( "PHAD_Q3_PROTOCOL_ID" ), 0 );
    EXPECT_EQ( exit_code, 1 );
    EXPECT_FALSE( std::filesystem::exists( out_dir ) );
    EXPECT_NE( readFile( stderr_path ).find( "INPUT_NOT_FOUND" ),
               std::string::npos );
#endif
  }

  TEST( GyroAlignmentCliTest,
        BinaryDefinedAndDynamicSymbolsExcludeGtAteRpeAndDataset )
  {
#ifdef __linux__
    ScopedDirectory   root{ "nm" };
    const auto        symbols = root.path / "symbols.txt";
    const auto        errors  = root.path / "nm-errors.txt";
    const std::string command =
        "nm -C --defined-only \"" + std::string{ PHAD_GYRO_ALIGN_PATH } +
        "\" > \"" + symbols.string() + "\" 2> \"" + errors.string() +
        "\" && nm -D -C \"" + std::string{ PHAD_GYRO_ALIGN_PATH } +
        "\" >> \"" + symbols.string() + "\" 2>> \"" + errors.string() +
        "\"";
    const int status = std::system( command.c_str() );
    ASSERT_TRUE( WIFEXITED( status ) );
    ASSERT_EQ( WEXITSTATUS( status ), 0 ) << readFile( errors );
    std::string text = readFile( symbols );
    std::transform( text.begin(), text.end(), text.begin(),
                    []( unsigned char c ) {
                      return static_cast<char>( std::tolower( c ) );
                    } );
    for ( const auto forbidden :
          { "groundtruth", "ground_truth", "phad::eval::computeate",
            "phad::eval::computerpe", "phad::io::dataset", "io_dataset",
            "euroc" } )
    {
      EXPECT_EQ( text.find( forbidden ), std::string::npos ) << forbidden;
    }
#endif
  }

}  // namespace
