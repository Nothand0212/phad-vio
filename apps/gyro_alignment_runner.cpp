#include "apps/gyro_alignment_runner.hpp"

#include <fcntl.h>
#include <openssl/evp.h>
#include <sys/stat.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/eval/tum_io.hpp"
#include "phad/sensor/imu_measurement.hpp"

#ifndef PHAD_Q3_DESIGN_COMMIT
#error "PHAD_Q3_DESIGN_COMMIT must be supplied by the build"
#endif
#ifndef PHAD_Q3_DESIGN_TREE
#error "PHAD_Q3_DESIGN_TREE must be supplied by the build"
#endif
#ifndef PHAD_Q3_DESIGN_BLOB
#error "PHAD_Q3_DESIGN_BLOB must be supplied by the build"
#endif
#ifndef PHAD_Q3_DESIGN_SHA256
#error "PHAD_Q3_DESIGN_SHA256 must be supplied by the build"
#endif
#ifndef PHAD_Q3_SOURCE_COMMIT
#error "PHAD_Q3_SOURCE_COMMIT must be supplied by the build"
#endif
#ifndef PHAD_Q3_SOURCE_TREE
#error "PHAD_Q3_SOURCE_TREE must be supplied by the build"
#endif
#ifndef PHAD_Q3_BUILD_TYPE
#error "PHAD_Q3_BUILD_TYPE must be supplied by the build"
#endif
#ifndef PHAD_Q3_COMPILER
#error "PHAD_Q3_COMPILER must be supplied by the build"
#endif
#ifndef PHAD_Q3_COMPILER_VERSION
#error "PHAD_Q3_COMPILER_VERSION must be supplied by the build"
#endif

namespace phad::apps
{
  namespace fs = std::filesystem;

  namespace
  {

    constexpr std::string_view kV1ProtocolId =
        "PHAD-M4-Q3-GYRO-ALIGN-V1";
    constexpr std::string_view kSyntheticProtocolId =
        "PHAD-M4-Q3-GYRO-ALIGN-SYNTHETIC-TEST-V1";
    constexpr std::array<std::string_view, 4> kInputNames{
        "est.tum", "diag.csv", "gyro_packets.csv", "gyro_samples.csv" };
    constexpr std::array<std::string_view, 4> kV1Hashes{
        "18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321",
        "1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb",
        "fbf574545e418fc19d100b7b05f79546a5336420bca60852770605b42702a3fe",
        "da35227b40a1ab47c94217b4210b5445d11927a864d6634f8b824812ccec0344" };
    constexpr std::string_view kDiagHeader =
        "timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,"
        "window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,"
        "num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,"
        "pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px,"
        "is_keyframe,num_disparity";
    constexpr std::string_view kPacketHeader =
        "packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,"
        "sum_dt_ns,interval_ns,status";
    constexpr std::string_view kSampleHeader =
        "packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,"
        "gyr_z_radps";
    constexpr std::string_view kBlocksHeader =
        "schema_version,block_index,t_start_ns,t_end_ns,segment_id,packet_count,"
        "r_zero_x_rad,r_zero_y_rad,r_zero_z_rad,r_fit_x_rad,r_fit_y_rad,"
        "r_fit_z_rad,loss_zero_rad2,loss_fit_rad2,improved\n";

    struct RunnerFailure final : std::runtime_error
    {
      RunnerFailure( GyroAlignmentErrorCode code_value, std::string detail_value )
          : std::runtime_error{ detail_value }, code{ code_value }, detail{ std::move( detail_value ) }
      {
      }

      GyroAlignmentErrorCode code;
      std::string            detail;
    };

    struct InputBytes
    {
      std::string   name;
      std::string   bytes;
      std::string   sha256;
      std::uint64_t size{};
    };

    class UniqueFd
    {
    public:
      explicit UniqueFd( int fd ) noexcept : m_fd{ fd } {}

      ~UniqueFd()
      {
        if ( m_fd >= 0 )
        {
          static_cast<void>( ::close( m_fd ) );
        }
      }

      UniqueFd( const UniqueFd& )            = delete;
      UniqueFd& operator=( const UniqueFd& ) = delete;

      [[nodiscard]] int get() const noexcept { return m_fd; }

    private:
      int m_fd;
    };

    struct DiagRow
    {
      std::int64_t                timestamp_ns{};
      GyroAlignmentEndpointStatus status{};
      std::uint32_t               segment_id{};
    };

    struct PacketRow
    {
      GyroAlignmentPacket packet;
      std::uint64_t       sample_count{};
    };

    struct SampleRow
    {
      std::uint64_t          packet_index{};
      std::uint64_t          sample_index{};
      sensor::ImuMeasurement sample;
    };

    [[nodiscard]] std::string_view errorName( GyroAlignmentErrorCode code )
    {
      constexpr std::array<std::string_view, 24> kNames{
          "INVALID_PROTOCOL_DESCRIPTOR", "INPUT_NOT_FOUND",
          "INPUT_NOT_REGULAR_FILE", "INPUT_HASH_MISMATCH", "INPUT_IO",
          "INPUT_GRAMMAR", "INPUT_SCHEMA", "UNKNOWN_ENUM", "DIAG_FAILED",
          "INDEX_ORDER", "COUNT_MISMATCH", "DURATION_MISMATCH",
          "ENDPOINT_MISMATCH", "JOIN_MISMATCH", "TIMESTAMP_OVERFLOW",
          "NONFINITE_INPUT", "INVALID_QUATERNION",
          "DIAG_POSE_CONTRADICTION", "Q2_INTEGRATION_ERROR",
          "NONFINITE_COMPUTATION", "OUTPUT_EXISTS", "OUTPUT_IO",
          "OUTPUT_PUBLISH", "OUTPUT_REHASH_MISMATCH" };
      return kNames.at( static_cast<std::size_t>( code ) );
    }

    [[nodiscard]] GyroAlignmentResult hardResult( GyroAlignmentErrorCode code,
                                                  std::string            detail )
    {
      return GyroAlignmentResult{
          .verdict  = { .status       = GyroAlignmentStatus::kHardError,
                        .exit_code    = 1,
                        .reason_codes = {},
                        .detail       = std::string{ errorName( code ) } +
                                  ": " + std::move( detail ) },
          .error    = code,
          .analysis = std::nullopt,
      };
    }

    [[nodiscard]] bool isLowerHex( std::string_view value,
                                   std::size_t      size )
    {
      return value.size() == size &&
             std::all_of( value.begin(), value.end(), []( char c ) {
               return ( c >= '0' && c <= '9' ) || ( c >= 'a' && c <= 'f' );
             } );
    }

    [[nodiscard]] bool isSafeText( std::string_view value )
    {
      return !value.empty() &&
             std::all_of( value.begin(), value.end(), []( unsigned char c ) {
               return c >= 0x20U && c <= 0x7eU;
             } );
    }

    [[nodiscard]] bool sameInputs(
        const std::vector<GyroAlignmentInputIdentity>& lhs,
        const std::vector<GyroAlignmentInputIdentity>& rhs )
    {
      if ( lhs.size() != rhs.size() )
      {
        return false;
      }
      for ( std::size_t i = 0; i < lhs.size(); ++i )
      {
        if ( lhs[ i ].basename != rhs[ i ].basename ||
             lhs[ i ].expected_sha256 != rhs[ i ].expected_sha256 )
        {
          return false;
        }
      }
      return true;
    }

    [[nodiscard]] bool sameProtocolIdentity(
        const GyroAlignmentProtocolIdentity& lhs,
        const GyroAlignmentProtocolIdentity& rhs )
    {
      return lhs.design_commit == rhs.design_commit &&
             lhs.design_tree == rhs.design_tree &&
             lhs.design_blob == rhs.design_blob &&
             lhs.design_sha256 == rhs.design_sha256;
    }

    [[nodiscard]] bool sameAnalyzerIdentity(
        const GyroAlignmentAnalyzerIdentity& lhs,
        const GyroAlignmentAnalyzerIdentity& rhs )
    {
      return lhs.source_commit == rhs.source_commit &&
             lhs.source_tree == rhs.source_tree &&
             lhs.build_type == rhs.build_type &&
             lhs.compiler == rhs.compiler &&
             lhs.compiler_version == rhs.compiler_version;
    }

    [[nodiscard]] bool sameProvenance(
        const GyroAlignmentProvenance& lhs,
        const GyroAlignmentProvenance& rhs )
    {
      return lhs.q1_source_run == rhs.q1_source_run &&
             lhs.q1_source_commit == rhs.q1_source_commit &&
             lhs.q1_git_tree_object == rhs.q1_git_tree_object &&
             lhs.q1_git_ls_tree_sha256 == rhs.q1_git_ls_tree_sha256 &&
             lhs.q1_meta_sha256 == rhs.q1_meta_sha256 &&
             lhs.q1_config_hash == rhs.q1_config_hash &&
             lhs.q1_input_manifest_v2_sha256 ==
                 rhs.q1_input_manifest_v2_sha256 &&
             lhs.q2_commit == rhs.q2_commit && lhs.q2_tree == rhs.q2_tree;
    }

    [[nodiscard]] bool sameDescriptor(
        const GyroAlignmentProtocolDescriptor& lhs,
        const GyroAlignmentProtocolDescriptor& rhs )
    {
      return lhs.protocol_id == rhs.protocol_id &&
             sameInputs( lhs.inputs, rhs.inputs ) &&
             sameProtocolIdentity( lhs.protocol_identity,
                                   rhs.protocol_identity ) &&
             sameAnalyzerIdentity( lhs.analyzer_identity,
                                   rhs.analyzer_identity ) &&
             sameProvenance( lhs.provenance, rhs.provenance );
    }

    [[nodiscard]] bool validSyntheticDescriptor(
        const GyroAlignmentProtocolDescriptor& descriptor )
    {
      if ( descriptor.protocol_id != kSyntheticProtocolId ||
           descriptor.inputs.size() != kInputNames.size() )
      {
        return false;
      }
      for ( std::size_t i = 0; i < kInputNames.size(); ++i )
      {
        if ( descriptor.inputs[ i ].basename != kInputNames[ i ] ||
             !isLowerHex( descriptor.inputs[ i ].expected_sha256, 64U ) )
        {
          return false;
        }
      }
      const auto& p = descriptor.protocol_identity;
      const auto& a = descriptor.analyzer_identity;
      const auto& v = descriptor.provenance;
      if ( !isLowerHex( p.design_commit, 40U ) ||
           !isLowerHex( p.design_tree, 40U ) ||
           !isLowerHex( p.design_blob, 40U ) ||
           !isLowerHex( p.design_sha256, 64U ) ||
           !isLowerHex( a.source_commit, 40U ) ||
           !isLowerHex( a.source_tree, 40U ) || !isSafeText( a.build_type ) ||
           !isSafeText( a.compiler ) || !isSafeText( a.compiler_version ) ||
           !isSafeText( v.q1_source_run ) ||
           !isLowerHex( v.q1_source_commit, 40U ) ||
           !isLowerHex( v.q1_git_tree_object, 40U ) ||
           !isLowerHex( v.q1_git_ls_tree_sha256, 64U ) ||
           !isLowerHex( v.q1_meta_sha256, 64U ) ||
           !isLowerHex( v.q1_config_hash, 8U ) ||
           !isLowerHex( v.q1_input_manifest_v2_sha256, 64U ) ||
           !isLowerHex( v.q2_commit, 40U ) || !isLowerHex( v.q2_tree, 40U ) )
      {
        return false;
      }
      const auto v1 = makeGyroAlignmentV1ProtocolDescriptor();
      return !sameInputs( descriptor.inputs, v1.inputs ) &&
             !sameProtocolIdentity( descriptor.protocol_identity,
                                    v1.protocol_identity ) &&
             !sameAnalyzerIdentity( descriptor.analyzer_identity,
                                    v1.analyzer_identity ) &&
             !sameProvenance( descriptor.provenance, v1.provenance );
    }

    [[nodiscard]] std::string sha256( std::string_view bytes )
    {
      std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
      std::size_t                                digest_size = digest.size();
      if ( EVP_Q_digest( nullptr, "SHA256", nullptr, bytes.data(),
                         bytes.size(), digest.data(), &digest_size ) != 1 ||
           digest_size != 32U )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                             "SHA-256 computation failed" };
      }
      std::ostringstream stream;
      stream << std::hex << std::setfill( '0' );
      for ( std::size_t i = 0; i < digest_size; ++i )
      {
        stream << std::setw( 2 ) << static_cast<unsigned int>( digest[ i ] );
      }
      return stream.str();
    }

    [[nodiscard]] int openDirectory( const fs::path& path )
    {
      int fd = -1;
      do
      {
        fd = ::open( path.c_str(), O_RDONLY | O_CLOEXEC | O_DIRECTORY );
      } while ( fd < 0 && errno == EINTR );
      if ( fd >= 0 )
      {
        return fd;
      }
      if ( errno == ENOENT || errno == ENOTDIR )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputNotFound,
                             "input directory does not exist" };
      }
      throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                           "failed to open input directory" };
    }

    [[nodiscard]] InputBytes readInput( int q1_fd, std::string name )
    {
      struct stat path_info
      {
      };
      int path_stat_result = -1;
      do
      {
        path_stat_result = ::fstatat( q1_fd, name.c_str(), &path_info,
                                      AT_SYMLINK_NOFOLLOW );
      } while ( path_stat_result < 0 && errno == EINTR );
      if ( path_stat_result < 0 )
      {
        if ( errno == ENOENT )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputNotFound,
                               "missing input " + name };
        }
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                             "failed to inspect input " + name };
      }
      if ( !S_ISREG( path_info.st_mode ) )
      {
        throw RunnerFailure{
            GyroAlignmentErrorCode::kInputNotRegularFile,
            "input is not a regular nonsymlink file: " + name };
      }

      int fd = -1;
      do
      {
        fd = ::openat( q1_fd, name.c_str(),
                       O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK );
      } while ( fd < 0 && errno == EINTR );
      if ( fd < 0 )
      {
        if ( errno == ENOENT )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputNotFound,
                               "missing input " + name };
        }
        if ( errno == ELOOP )
        {
          throw RunnerFailure{
              GyroAlignmentErrorCode::kInputNotRegularFile,
              "input is not a regular nonsymlink file: " + name };
        }
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                             "failed to open input " + name };
      }
      const UniqueFd input_fd{ fd };

      struct stat info
      {
      };
      int stat_result = -1;
      do
      {
        stat_result = ::fstat( input_fd.get(), &info );
      } while ( stat_result < 0 && errno == EINTR );
      if ( stat_result < 0 )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                             "failed to inspect opened input " + name };
      }
      if ( !S_ISREG( info.st_mode ) )
      {
        throw RunnerFailure{
            GyroAlignmentErrorCode::kInputNotRegularFile,
            "input is not a regular nonsymlink file: " + name };
      }
      if ( info.st_dev != path_info.st_dev || info.st_ino != path_info.st_ino )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                             "input identity changed while opening " + name };
      }
      if ( info.st_size < 0 )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                             "input size is negative for " + name };
      }

      InputBytes input;
      input.name             = std::move( name );
      const auto hinted_size = static_cast<std::uintmax_t>( info.st_size );
      if ( hinted_size > input.bytes.max_size() ||
           hinted_size > std::numeric_limits<std::uint64_t>::max() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                             "input size is not representable" };
      }
      try
      {
        input.bytes.reserve( static_cast<std::size_t>( hinted_size ) );
      }
      catch ( const std::exception& )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                             "failed to reserve input buffer" };
      }

      std::array<char, 64U * 1024U> chunk{};
      while ( true )
      {
        ssize_t count = -1;
        do
        {
          count = ::read( input_fd.get(), chunk.data(), chunk.size() );
        } while ( count < 0 && errno == EINTR );
        if ( count < 0 )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                               "failed to read input " + input.name };
        }
        if ( count == 0 )
        {
          break;
        }
        const auto read_size = static_cast<std::size_t>( count );
        if ( read_size > input.bytes.max_size() - input.bytes.size() ||
             read_size > std::numeric_limits<std::uint64_t>::max() -
                             input.size )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                               "input size is not representable" };
        }
        try
        {
          input.bytes.append( chunk.data(), read_size );
        }
        catch ( const std::exception& )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputIo,
                               "failed to grow input buffer" };
        }
        input.size += static_cast<std::uint64_t>( read_size );
      }
      return input;
    }

    [[nodiscard]] std::vector<std::string_view> dataLines(
        std::string_view bytes )
    {
      std::vector<std::string_view> lines;
      std::size_t                   begin = 0U;
      while ( begin < bytes.size() )
      {
        const std::size_t end  = bytes.find( '\n', begin );
        auto              line = end == std::string_view::npos
                                     ? bytes.substr( begin )
                                     : bytes.substr( begin, end - begin );
        if ( end != std::string_view::npos && !line.empty() &&
             line.back() == '\r' )
        {
          line.remove_suffix( 1U );
        }
        if ( line.find( '\r' ) != std::string_view::npos )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                               "line endings must be LF or CRLF" };
        }
        lines.push_back( line );
        begin = end == std::string_view::npos ? bytes.size() : end + 1U;
      }
      return lines;
    }

    [[nodiscard]] std::vector<std::string_view> csvFields(
        std::string_view line, std::size_t expected )
    {
      if ( line.empty() || line.find( '"' ) != std::string_view::npos )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "CSV row is blank or quoted" };
      }
      std::vector<std::string_view> fields;
      std::size_t                   begin = 0U;
      while ( true )
      {
        const auto end = line.find( ',', begin );
        fields.push_back( end == std::string_view::npos
                              ? line.substr( begin )
                              : line.substr( begin, end - begin ) );
        if ( end == std::string_view::npos )
        {
          break;
        }
        begin = end + 1U;
      }
      if ( fields.size() != expected )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "CSV row has the wrong column count" };
      }
      if ( std::any_of( fields.begin(), fields.end(),
                        []( std::string_view field ) {
                          return field.empty();
                        } ) )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "CSV field must not be empty" };
      }
      return fields;
    }

    [[nodiscard]] std::uint64_t parseUint( std::string_view text )
    {
      if ( text.empty() || ( text.size() > 1U && text.front() == '0' ) )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "expected canonical unsigned decimal" };
      }
      std::uint64_t value  = 0U;
      const auto    result = std::from_chars( text.data(),
                                              text.data() + text.size(), value );
      if ( result.ec != std::errc{} ||
           result.ptr != text.data() + text.size() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "expected canonical unsigned decimal" };
      }
      return value;
    }

    [[nodiscard]] std::int64_t parseInt( std::string_view text )
    {
      if ( text.empty() || text.front() == '+' || text == "-0" ||
           ( text.size() > 1U && text.front() == '0' ) ||
           ( text.size() > 2U && text[ 0 ] == '-' && text[ 1 ] == '0' ) )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "expected canonical signed decimal" };
      }
      std::int64_t value  = 0;
      const auto   result = std::from_chars( text.data(),
                                             text.data() + text.size(), value );
      if ( result.ec == std::errc::result_out_of_range )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kTimestampOverflow,
                             "signed integer is outside int64 range" };
      }
      if ( result.ec != std::errc{} ||
           result.ptr != text.data() + text.size() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "expected canonical signed decimal" };
      }
      return value;
    }

    [[nodiscard]] double parseFinite( std::string_view text )
    {
      double     value  = 0.0;
      const auto result = std::from_chars( text.data(),
                                           text.data() + text.size(), value,
                                           std::chars_format::general );
      if ( result.ec != std::errc{} ||
           result.ptr != text.data() + text.size() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "expected a floating-point token" };
      }
      if ( !std::isfinite( value ) )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kNonfiniteInput,
                             "floating-point input must be finite" };
      }
      return value;
    }

    [[nodiscard]] bool parseBool( std::string_view text )
    {
      if ( text == "0" )
      {
        return false;
      }
      if ( text == "1" )
      {
        return true;
      }
      throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                           "boolean field must be 0 or 1" };
    }

    [[nodiscard]] std::int64_t parseTumTimestamp( std::string_view text )
    {
      const auto dot = text.find( '.' );
      if ( dot == std::string_view::npos ||
           text.find( '.', dot + 1U ) != std::string_view::npos ||
           text.size() - dot - 1U != 9U || text == "-0.000000000" )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "TUM timestamp is not canonical nanoseconds" };
      }
      const auto seconds_text  = text.substr( 0U, dot );
      const auto fraction_text = text.substr( dot + 1U );
      if ( fraction_text.empty() ||
           !std::all_of( fraction_text.begin(), fraction_text.end(),
                         []( char c ) { return c >= '0' && c <= '9'; } ) )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "TUM fraction must have nine decimal digits" };
      }
      const bool negative = !seconds_text.empty() &&
                            seconds_text.front() == '-';
      const auto magnitude = negative ? seconds_text.substr( 1U )
                                      : seconds_text;
      if ( magnitude.empty() ||
           ( magnitude.size() > 1U && magnitude.front() == '0' ) ||
           !std::all_of( magnitude.begin(), magnitude.end(), []( char c ) {
             return c >= '0' && c <= '9';
           } ) )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "TUM seconds field is not canonical" };
      }
      std::uint64_t seconds  = 0U;
      std::uint64_t fraction = 0U;
      const auto    seconds_result =
          std::from_chars( magnitude.data(), magnitude.data() + magnitude.size(),
                           seconds );
      const auto fraction_result = std::from_chars(
          fraction_text.data(), fraction_text.data() + fraction_text.size(),
          fraction );
      if ( seconds_result.ec == std::errc::result_out_of_range )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kTimestampOverflow,
                             "TUM timestamp is outside int64 range" };
      }
      if ( seconds_result.ec != std::errc{} ||
           fraction_result.ec != std::errc{} )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "invalid TUM timestamp" };
      }
      constexpr std::uint64_t kNsPerSecond  = 1'000'000'000ULL;
      const auto              max_magnitude = negative
                                                  ? ( std::uint64_t{ 1 } << 63U )
                                                  : static_cast<std::uint64_t>(
                                           std::numeric_limits<std::int64_t>::max() );
      if ( seconds > max_magnitude / kNsPerSecond )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kTimestampOverflow,
                             "TUM timestamp is outside int64 range" };
      }
      const auto seconds_ns = seconds * kNsPerSecond;
      if ( seconds_ns > max_magnitude - fraction )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kTimestampOverflow,
                             "TUM timestamp is outside int64 range" };
      }
      const auto magnitude_ns = seconds_ns + fraction;
      if ( negative && magnitude_ns == ( std::uint64_t{ 1 } << 63U ) )
      {
        return std::numeric_limits<std::int64_t>::min();
      }
      const auto signed_value = static_cast<std::int64_t>( magnitude_ns );
      return negative ? -signed_value : signed_value;
    }

    [[nodiscard]] std::vector<std::string_view> tumFields(
        std::string_view line )
    {
      if ( line.empty() || line.front() == '#' )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputGrammar,
                             "Q3 TUM forbids comments and blank rows" };
      }
      std::vector<std::string_view> fields;
      std::size_t                   i = 0U;
      while ( i < line.size() )
      {
        while ( i < line.size() && ( line[ i ] == ' ' || line[ i ] == '\t' ) )
        {
          ++i;
        }
        if ( i == line.size() )
        {
          break;
        }
        const auto begin = i;
        while ( i < line.size() && line[ i ] != ' ' && line[ i ] != '\t' )
        {
          ++i;
        }
        fields.push_back( line.substr( begin, i - begin ) );
      }
      if ( fields.size() != 8U )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "TUM row must contain exactly eight fields" };
      }
      return fields;
    }

    void validateTumGrammar( std::string_view bytes )
    {
      const auto lines = dataLines( bytes );
      if ( lines.empty() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "TUM input is empty" };
      }
      for ( const auto line : lines )
      {
        const auto fields = tumFields( line );
        static_cast<void>( parseTumTimestamp( fields.front() ) );
      }
    }

    void validateCsvGrammar( std::string_view bytes,
                             std::string_view header,
                             std::size_t      columns,
                             std::string_view name )
    {
      const auto lines = dataLines( bytes );
      if ( lines.empty() || lines.front() != header )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             std::string{ name } + " header mismatch" };
      }
      for ( std::size_t i = 1U; i < lines.size(); ++i )
      {
        static_cast<void>( csvFields( lines[ i ], columns ) );
      }
    }

    void validateTum( std::string_view bytes )
    {
      const auto lines = dataLines( bytes );
      if ( lines.empty() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "TUM input is empty" };
      }
      std::optional<std::int64_t> previous;
      for ( const auto line : lines )
      {
        const auto fields    = tumFields( line );
        const auto timestamp = parseTumTimestamp( fields[ 0 ] );
        if ( previous.has_value() && timestamp <= *previous )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kIndexOrder,
                               "TUM timestamps must strictly increase" };
        }
        previous = timestamp;
        std::array<double, 7> values{};
        for ( std::size_t i = 0; i < values.size(); ++i )
        {
          values[ i ] = parseFinite( fields[ i + 1U ] );
        }
        const Eigen::Quaterniond q{ values[ 6 ], values[ 3 ], values[ 4 ],
                                    values[ 5 ] };
        if ( !std::isfinite( q.norm() ) ||
             std::abs( q.norm() - 1.0 ) > 1.0e-3 )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInvalidQuaternion,
                               "TUM quaternion must have unit norm" };
        }
      }
    }

    [[nodiscard]] std::vector<DiagRow> parseDiag( std::string_view bytes )
    {
      const auto lines = dataLines( bytes );
      if ( lines.empty() || lines.front() != kDiagHeader )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "diag.csv header mismatch" };
      }
      std::vector<DiagRow> rows;
      for ( std::size_t i = 1U; i < lines.size(); ++i )
      {
        const auto fields = csvFields( lines[ i ], 20U );
        DiagRow    row;
        row.timestamp_ns = parseInt( fields[ 0 ] );
        if ( fields[ 1 ] == "ok" )
        {
          row.status = GyroAlignmentEndpointStatus::kOk;
        }
        else if ( fields[ 1 ] == "rejected" )
        {
          row.status = GyroAlignmentEndpointStatus::kRejected;
        }
        else if ( fields[ 1 ] == "failed" )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kDiagFailed,
                               "diag.csv contains failed status" };
        }
        else
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kUnknownEnum,
                               "unknown diag status" };
        }
        for ( const auto index : { 2U, 3U, 4U, 6U, 7U, 10U, 11U, 15U, 16U,
                                   19U } )
        {
          static_cast<void>( parseUint( fields[ index ] ) );
        }
        for ( const auto index : { 5U, 14U, 18U } )
        {
          static_cast<void>( parseBool( fields[ index ] ) );
        }
        for ( const auto index : { 8U, 9U, 12U, 17U } )
        {
          static_cast<void>( parseFinite( fields[ index ] ) );
        }
        const auto segment = parseUint( fields[ 13 ] );
        if ( segment > std::numeric_limits<std::uint32_t>::max() )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                               "segment id exceeds uint32 range" };
        }
        row.segment_id = static_cast<std::uint32_t>( segment );
        if ( !rows.empty() && row.timestamp_ns <= rows.back().timestamp_ns )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kIndexOrder,
                               "diag timestamps must strictly increase" };
        }
        rows.push_back( row );
      }
      if ( rows.empty() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "diag.csv has no data rows" };
      }
      return rows;
    }

    [[nodiscard]] std::int64_t checkedDifference( std::int64_t           lhs,
                                                  std::int64_t           rhs,
                                                  GyroAlignmentErrorCode code,
                                                  std::string            detail )
    {
      if ( ( rhs > 0 && lhs < std::numeric_limits<std::int64_t>::min() + rhs ) ||
           ( rhs < 0 && lhs > std::numeric_limits<std::int64_t>::max() + rhs ) )
      {
        throw RunnerFailure{ code, std::move( detail ) };
      }
      return lhs - rhs;
    }

    [[nodiscard]] std::int64_t checkedAdd( std::int64_t           lhs,
                                           std::int64_t           rhs,
                                           GyroAlignmentErrorCode code,
                                           std::string            detail )
    {
      if ( ( rhs > 0 && lhs > std::numeric_limits<std::int64_t>::max() - rhs ) ||
           ( rhs < 0 && lhs < std::numeric_limits<std::int64_t>::min() - rhs ) )
      {
        throw RunnerFailure{ code, std::move( detail ) };
      }
      return lhs + rhs;
    }

    [[nodiscard]] std::vector<PacketRow> parsePackets(
        std::string_view bytes )
    {
      const auto lines = dataLines( bytes );
      if ( lines.empty() || lines.front() != kPacketHeader )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "gyro_packets.csv header mismatch" };
      }
      std::vector<PacketRow> rows;
      for ( std::size_t i = 1U; i < lines.size(); ++i )
      {
        const auto fields = csvFields( lines[ i ], 9U );
        PacketRow  row;
        row.packet.packet_index = parseUint( fields[ 0 ] );
        row.packet.t_prev_ns    = parseInt( fields[ 1 ] );
        row.packet.t_cur_ns     = parseInt( fields[ 2 ] );
        const auto segment      = parseUint( fields[ 3 ] );
        if ( segment > std::numeric_limits<std::uint32_t>::max() )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                               "packet segment id exceeds uint32 range" };
        }
        row.packet.segment_id  = static_cast<std::uint32_t>( segment );
        row.packet.imu_gap     = parseBool( fields[ 4 ] );
        row.sample_count       = parseUint( fields[ 5 ] );
        row.packet.sum_dt_ns   = parseInt( fields[ 6 ] );
        row.packet.interval_ns = parseInt( fields[ 7 ] );
        if ( fields[ 8 ] == "first_zero" )
        {
          row.packet.status = GyroAlignmentPacketStatus::kFirstZero;
        }
        else if ( fields[ 8 ] == "valid" )
        {
          row.packet.status = GyroAlignmentPacketStatus::kValid;
        }
        else if ( fields[ 8 ] == "gap" )
        {
          row.packet.status = GyroAlignmentPacketStatus::kGap;
        }
        else if ( fields[ 8 ] == "empty_nonfirst" )
        {
          row.packet.status = GyroAlignmentPacketStatus::kEmptyNonfirst;
        }
        else
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kUnknownEnum,
                               "unknown packet status" };
        }
        if ( row.packet.packet_index != rows.size() )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kIndexOrder,
                               "packet indices must start at zero and be continuous" };
        }
        if ( !rows.empty() &&
             row.packet.t_cur_ns <= rows.back().packet.t_cur_ns )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kIndexOrder,
                               "packet current timestamps must strictly increase" };
        }
        rows.push_back( std::move( row ) );
      }
      if ( rows.empty() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "gyro_packets.csv has no data rows" };
      }
      return rows;
    }

    [[nodiscard]] std::vector<SampleRow> parseSamples(
        std::string_view bytes )
    {
      const auto lines = dataLines( bytes );
      if ( lines.empty() || lines.front() != kSampleHeader )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             "gyro_samples.csv header mismatch" };
      }
      std::vector<SampleRow>       rows;
      std::optional<std::uint64_t> previous_packet;
      std::uint64_t                expected_sample = 0U;
      for ( std::size_t i = 1U; i < lines.size(); ++i )
      {
        const auto fields = csvFields( lines[ i ], 6U );
        SampleRow  row;
        row.packet_index     = parseUint( fields[ 0 ] );
        row.sample_index     = parseUint( fields[ 1 ] );
        const auto timestamp = parseInt( fields[ 2 ] );
        row.sample           = sensor::ImuMeasurement{
                      .timestamp  = common::Timestamp{ timestamp },
                      .accel_mps2 = { 0.0, 0.0, 0.0 },
                      .gyro_radps = { parseFinite( fields[ 3 ] ),
                                      parseFinite( fields[ 4 ] ),
                                      parseFinite( fields[ 5 ] ) },
        };
        if ( !previous_packet.has_value() ||
             row.packet_index != *previous_packet )
        {
          if ( previous_packet.has_value() &&
               row.packet_index <= *previous_packet )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kIndexOrder,
                                 "sample packet groups must strictly increase" };
          }
          previous_packet = row.packet_index;
          expected_sample = 0U;
        }
        if ( row.sample_index != expected_sample )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kIndexOrder,
                               "sample indices must start at zero and be continuous" };
        }
        ++expected_sample;
        rows.push_back( std::move( row ) );
      }
      return rows;
    }

    void attachAndValidateSamples( std::vector<PacketRow>&       packets,
                                   const std::vector<SampleRow>& samples )
    {
      for ( const auto& row : samples )
      {
        if ( row.packet_index >= packets.size() )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kCountMismatch,
                               "sample references an unknown packet" };
        }
        packets[ static_cast<std::size_t>( row.packet_index ) ]
            .packet.samples.push_back( row.sample );
      }

      for ( std::size_t i = 0; i < packets.size(); ++i )
      {
        auto& row    = packets[ i ];
        auto& packet = row.packet;
        if ( packet.samples.size() != row.sample_count )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kCountMismatch,
                               "packet sample_count does not match samples" };
        }
        std::int64_t sum_dt_ns = 0;
        for ( std::size_t j = 1U; j < packet.samples.size(); ++j )
        {
          const auto previous =
              packet.samples[ j - 1U ].timestamp.nanoseconds();
          const auto current = packet.samples[ j ].timestamp.nanoseconds();
          if ( current <= previous )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kIndexOrder,
                                 "sample timestamps must strictly increase" };
          }
          const auto dt = checkedDifference(
              current, previous, GyroAlignmentErrorCode::kTimestampOverflow,
              "sample timestamp difference overflow" );
          sum_dt_ns = checkedAdd(
              sum_dt_ns, dt, GyroAlignmentErrorCode::kDurationMismatch,
              "sample duration sum overflow" );
        }
        if ( sum_dt_ns != packet.sum_dt_ns )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kDurationMismatch,
                               "sum_dt_ns does not match sample deltas" };
        }

        if ( i == 0U )
        {
          if ( packet.status != GyroAlignmentPacketStatus::kFirstZero ||
               packet.t_prev_ns != packet.t_cur_ns || packet.imu_gap ||
               !packet.samples.empty() )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kCountMismatch,
                                 "packet zero must be the first_zero sentinel" };
          }
          if ( packet.sum_dt_ns != 0 || packet.interval_ns != 0 )
          {
            throw RunnerFailure{
                GyroAlignmentErrorCode::kDurationMismatch,
                "first_zero duration fields must both be zero" };
          }
          continue;
        }
        if ( packet.status == GyroAlignmentPacketStatus::kFirstZero )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                               "first_zero may only appear at packet zero" };
        }
        const auto interval = checkedDifference(
            packet.t_cur_ns, packet.t_prev_ns,
            GyroAlignmentErrorCode::kTimestampOverflow,
            "packet interval subtraction overflow" );
        if ( interval <= 0 )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kEndpointMismatch,
                               "nonfirst packet endpoints must increase" };
        }
        if ( packet.interval_ns != interval )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kDurationMismatch,
                               "interval_ns does not match packet endpoints" };
        }
        if ( packet.status == GyroAlignmentPacketStatus::kValid )
        {
          if ( packet.imu_gap )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                                 "valid packet must have imu_gap=0" };
          }
          if ( packet.samples.size() < 2U )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kCountMismatch,
                                 "valid packet requires at least two samples" };
          }
          if ( packet.sum_dt_ns != packet.interval_ns )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kDurationMismatch,
                                 "valid packet duration must fill interval" };
          }
          if ( packet.samples.front().timestamp.nanoseconds() !=
                   packet.t_prev_ns ||
               packet.samples.back().timestamp.nanoseconds() != packet.t_cur_ns )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kEndpointMismatch,
                                 "valid packet samples must match endpoints" };
          }
        }
        else if ( packet.status == GyroAlignmentPacketStatus::kGap )
        {
          if ( !packet.imu_gap )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                                 "gap packet must have imu_gap=1" };
          }
        }
        else
        {
          if ( packet.imu_gap || !packet.samples.empty() ||
               packet.sum_dt_ns != 0 )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kCountMismatch,
                                 "empty_nonfirst packet must contain no samples" };
          }
        }
      }
    }

    [[nodiscard]] GyroAlignmentInput parseAndJoin(
        const std::array<InputBytes, 4>& inputs )
    {
      validateTumGrammar( inputs[ 0 ].bytes );
      validateCsvGrammar( inputs[ 1 ].bytes, kDiagHeader, 20U, "diag.csv" );
      validateCsvGrammar( inputs[ 2 ].bytes, kPacketHeader, 9U,
                          "gyro_packets.csv" );
      validateCsvGrammar( inputs[ 3 ].bytes, kSampleHeader, 6U,
                          "gyro_samples.csv" );
      validateTum( inputs[ 0 ].bytes );
      auto trajectory = eval::readTumBytes( inputs[ 0 ].bytes, inputs[ 0 ].name );
      if ( !trajectory )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kInputSchema,
                             trajectory.error().describe() };
      }
      const auto diag    = parseDiag( inputs[ 1 ].bytes );
      auto       packets = parsePackets( inputs[ 2 ].bytes );
      const auto samples = parseSamples( inputs[ 3 ].bytes );
      attachAndValidateSamples( packets, samples );

      if ( diag.size() != packets.size() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kJoinMismatch,
                             "diag and packet current endpoints are not one-to-one" };
      }
      for ( std::size_t i = 0; i < diag.size(); ++i )
      {
        if ( diag[ i ].timestamp_ns != packets[ i ].packet.t_cur_ns ||
             diag[ i ].segment_id != packets[ i ].packet.segment_id )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kJoinMismatch,
                               "packet current endpoint does not match diag" };
        }
      }

      GyroAlignmentInput result;
      result.endpoints.reserve( diag.size() );
      for ( const auto& row : diag )
      {
        result.endpoints.push_back( GyroAlignmentEndpoint{
            .timestamp_ns = row.timestamp_ns,
            .status       = row.status,
            .segment_id   = row.segment_id,
            .q_WB         = std::nullopt,
        } );
      }

      std::size_t diag_index = 0U;
      for ( const auto& pose : trajectory.value().poses() )
      {
        const auto timestamp = pose.timestamp.nanoseconds();
        while ( diag_index < result.endpoints.size() &&
                result.endpoints[ diag_index ].timestamp_ns < timestamp )
        {
          ++diag_index;
        }
        if ( diag_index == result.endpoints.size() ||
             result.endpoints[ diag_index ].timestamp_ns != timestamp )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kJoinMismatch,
                               "pose has no unique diag endpoint" };
        }
        auto& endpoint = result.endpoints[ diag_index ];
        if ( endpoint.status != GyroAlignmentEndpointStatus::kOk ||
             endpoint.q_WB.has_value() )
        {
          throw RunnerFailure{
              GyroAlignmentErrorCode::kDiagPoseContradiction,
              "only a unique ok diag endpoint may carry a pose" };
        }
        endpoint.q_WB = Eigen::Quaterniond{ pose.T_W_B.linear() }.normalized();
      }

      for ( std::size_t i = 0; i < packets.size(); ++i )
      {
        const auto& packet = packets[ i ].packet;
        if ( packet.status == GyroAlignmentPacketStatus::kValid && i > 0U )
        {
          if ( packet.t_prev_ns != result.endpoints[ i - 1U ].timestamp_ns )
          {
            throw RunnerFailure{ GyroAlignmentErrorCode::kJoinMismatch,
                                 "valid packet previous endpoint is not adjacent" };
          }
        }
        result.packets.push_back( packet );
      }
      return result;
    }

    [[nodiscard]] std::string_view statusName( GyroAlignmentStatus status )
    {
      switch ( status )
      {
        case GyroAlignmentStatus::kPass:
          return "PASS";
        case GyroAlignmentStatus::kHypothesisFail:
          return "HYPOTHESIS_FAIL";
        case GyroAlignmentStatus::kInconclusive:
          return "INCONCLUSIVE";
        case GyroAlignmentStatus::kHardError:
          return "HARD_ERROR";
      }
      throw RunnerFailure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                           "unknown analyzer verdict" };
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
      throw RunnerFailure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                           "unknown fit status" };
    }

    [[nodiscard]] std::string_view reasonName( GyroAlignmentReason reason )
    {
      switch ( reason )
      {
        case GyroAlignmentReason::kFitSupport:
          return "FIT_SUPPORT";
        case GyroAlignmentReason::kValidationSupport:
          return "VALIDATION_SUPPORT";
        case GyroAlignmentReason::kFitRank:
          return "FIT_RANK";
        case GyroAlignmentReason::kFitCondition:
          return "FIT_CONDITION";
        case GyroAlignmentReason::kFitObservability:
          return "FIT_OBSERVABILITY";
        case GyroAlignmentReason::kPrefixNonlinearLoss:
          return "PREFIX_NONLINEAR_LOSS";
        case GyroAlignmentReason::kValidationAggregate:
          return "VALIDATION_AGGREGATE";
        case GyroAlignmentReason::kValidationFraction:
          return "VALIDATION_FRACTION";
        case GyroAlignmentReason::kValidationAxis:
          return "VALIDATION_AXIS";
        case GyroAlignmentReason::kHalfStability:
          return "HALF_STABILITY";
      }
      throw RunnerFailure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                           "unknown verdict reason" };
    }

    [[nodiscard]] nlohmann::ordered_json vectorJson(
        const std::optional<Eigen::Vector3d>& value )
    {
      if ( !value.has_value() )
      {
        return nullptr;
      }
      return nlohmann::ordered_json::array(
          { value->x(), value->y(), value->z() } );
    }

    template <typename T>
    [[nodiscard]] nlohmann::ordered_json optionalJson(
        const std::optional<T>& value )
    {
      if ( !value.has_value() )
      {
        return nullptr;
      }
      nlohmann::ordered_json scalar;
      scalar = *value;
      return scalar;
    }

    [[nodiscard]] nlohmann::ordered_json protocolIdentityJson(
        const GyroAlignmentProtocolIdentity& identity )
    {
      nlohmann::ordered_json value;
      value[ "design_commit" ] = identity.design_commit;
      value[ "design_tree" ]   = identity.design_tree;
      value[ "design_blob" ]   = identity.design_blob;
      value[ "design_sha256" ] = identity.design_sha256;
      return value;
    }

    [[nodiscard]] nlohmann::ordered_json analyzerIdentityJson(
        const GyroAlignmentAnalyzerIdentity& identity )
    {
      nlohmann::ordered_json value;
      value[ "source_commit" ]    = identity.source_commit;
      value[ "source_tree" ]      = identity.source_tree;
      value[ "build_type" ]       = identity.build_type;
      value[ "compiler" ]         = identity.compiler;
      value[ "compiler_version" ] = identity.compiler_version;
      return value;
    }

    [[nodiscard]] nlohmann::ordered_json provenanceJson(
        const GyroAlignmentProvenance& provenance )
    {
      nlohmann::ordered_json value;
      value[ "q1_source_run" ]      = provenance.q1_source_run;
      value[ "q1_source_commit" ]   = provenance.q1_source_commit;
      value[ "q1_git_tree_object" ] = provenance.q1_git_tree_object;
      value[ "q1_git_ls_tree_sha256" ] =
          provenance.q1_git_ls_tree_sha256;
      value[ "q1_meta_sha256" ] = provenance.q1_meta_sha256;
      value[ "q1_config_hash" ] = provenance.q1_config_hash;
      value[ "q1_input_manifest_v2_sha256" ] =
          provenance.q1_input_manifest_v2_sha256;
      value[ "q2_commit" ] = provenance.q2_commit;
      value[ "q2_tree" ]   = provenance.q2_tree;
      return value;
    }

    [[nodiscard]] nlohmann::ordered_json inputsJson(
        const std::array<InputBytes, 4>& inputs )
    {
      auto value = nlohmann::ordered_json::array();
      for ( const auto& input : inputs )
      {
        nlohmann::ordered_json item;
        item[ "name" ]       = input.name;
        item[ "size_bytes" ] = input.size;
        item[ "sha256" ]     = input.sha256;
        value.push_back( std::move( item ) );
      }
      return value;
    }

    [[nodiscard]] nlohmann::ordered_json fitJson(
        const GyroAlignmentFit& fit )
    {
      nlohmann::ordered_json value;
      value[ "status" ]               = fitStatusName( fit.status );
      value[ "bias_radps" ]           = vectorJson( fit.bias_radps );
      value[ "singular_values" ]      = vectorJson( fit.singular_values );
      value[ "rank" ]                 = optionalJson( fit.rank );
      value[ "condition" ]            = optionalJson( fit.condition );
      value[ "eligible_duration_ns" ] = fit.eligible_duration_ns;
      value[ "packet_count" ]         = fit.packet_count;
      value[ "loss_zero_rad2" ]       = optionalJson( fit.loss_zero_rad2 );
      value[ "loss_fit_rad2" ]        = optionalJson( fit.loss_fit_rad2 );
      value[ "nonlinear_nonincrease" ] =
          optionalJson( fit.nonlinear_nonincrease );
      return value;
    }

    [[nodiscard]] nlohmann::ordered_json verdictJson(
        const GyroAlignmentVerdict& verdict )
    {
      nlohmann::ordered_json value;
      value[ "status" ]    = statusName( verdict.status );
      value[ "exit_code" ] = verdict.exit_code;
      auto reasons         = nlohmann::ordered_json::array();
      for ( const auto reason : verdict.reason_codes )
      {
        reasons.push_back( reasonName( reason ) );
      }
      value[ "reason_codes" ] = std::move( reasons );
      value[ "detail" ]       = verdict.detail;
      return value;
    }

    [[nodiscard]] std::string summaryBytes(
        const GyroAlignmentProtocolDescriptor& descriptor,
        const std::array<InputBytes, 4>&       inputs,
        const GyroAlignmentResult&             result )
    {
      if ( !result.analysis.has_value() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                             "completed analyzer result has no analysis" };
      }
      const auto&            analysis = *result.analysis;
      nlohmann::ordered_json root;
      root[ "schema_version" ] = "phad.gyro_alignment.v1";
      root[ "protocol_id" ]    = descriptor.protocol_id;
      root[ "protocol_identity" ] =
          protocolIdentityJson( descriptor.protocol_identity );
      root[ "analyzer_identity" ] =
          analyzerIdentityJson( descriptor.analyzer_identity );
      nlohmann::ordered_json claim;
      claim[ "claim" ]               = "frozen_visual_proxy_suffix_rotation_consistency";
      claim[ "offline_future_data" ] = true;
      claim[ "physical_bias" ]       = false;
      claim[ "online_initializer" ]  = false;
      claim[ "product_benefit" ]     = false;
      claim[ "bias_role" ]           = "visual_posterior_aligned_nuisance";
      claim[ "pass_authority" ]      = "q4_controlled_factor_plan_only";
      root[ "summary" ]              = std::move( claim );
      root[ "provenance" ]           = provenanceJson( descriptor.provenance );
      root[ "inputs" ]               = inputsJson( inputs );

      nlohmann::ordered_json ranges;
      ranges[ "t0_ns" ]      = analysis.ranges.t0_ns;
      ranges[ "t_mid_ns" ]   = analysis.ranges.t_mid_ns;
      ranges[ "t_split_ns" ] = analysis.ranges.t_split_ns;
      ranges[ "validation_last_end_ns" ] =
          optionalJson( analysis.ranges.validation_last_end_ns );
      root[ "ranges" ] = std::move( ranges );

      nlohmann::ordered_json eligibility;
      eligibility[ "packet_total_nonfirst" ] =
          analysis.eligibility.packet_total_nonfirst;
      eligibility[ "packet_structurally_eligible" ] =
          analysis.eligibility.packet_structurally_eligible;
      eligibility[ "packet_fit_eligible" ] =
          analysis.eligibility.packet_fit_eligible;
      nlohmann::ordered_json interval_counts;
      interval_counts[ "gap" ] =
          analysis.eligibility.interval_excluded_counts.gap;
      interval_counts[ "empty_nonfirst" ] =
          analysis.eligibility.interval_excluded_counts.empty_nonfirst;
      interval_counts[ "diag_rejected_no_pose" ] =
          analysis.eligibility.interval_excluded_counts.diag_rejected_no_pose;
      interval_counts[ "segment_boundary" ] =
          analysis.eligibility.interval_excluded_counts.segment_boundary;
      interval_counts[ "near_pi" ] =
          analysis.eligibility.interval_excluded_counts.near_pi;
      eligibility[ "interval_excluded_counts" ] = std::move( interval_counts );
      eligibility[ "validation_slots_total" ] =
          analysis.eligibility.validation_slots_total;
      eligibility[ "validation_blocks_eligible" ] =
          optionalJson( analysis.eligibility.validation_blocks_eligible );
      nlohmann::ordered_json block_counts;
      block_counts[ "incomplete_calendar_block" ] = optionalJson(
          analysis.eligibility.block_excluded_counts.incomplete_calendar_block );
      block_counts[ "near_pi" ] = optionalJson(
          analysis.eligibility.block_excluded_counts.near_pi );
      eligibility[ "block_excluded_counts" ] = std::move( block_counts );
      root[ "eligibility" ]                  = std::move( eligibility );

      nlohmann::ordered_json fits;
      fits[ "full" ]  = fitJson( analysis.full );
      fits[ "early" ] = fitJson( analysis.early );
      fits[ "late" ]  = fitJson( analysis.late );
      root[ "fit" ]   = std::move( fits );

      nlohmann::ordered_json support;
      support[ "full_duration_ns" ]  = analysis.support.full_duration_ns;
      support[ "early_duration_ns" ] = analysis.support.early_duration_ns;
      support[ "late_duration_ns" ]  = analysis.support.late_duration_ns;
      support[ "validation_blocks" ] =
          optionalJson( analysis.support.validation_blocks );
      support[ "sufficient" ] = optionalJson( analysis.support.sufficient );
      root[ "support" ]       = std::move( support );

      nlohmann::ordered_json gates;
      gates[ "evaluated" ] = analysis.gates.evaluated;
      gates[ "validation_loss_zero_rad2" ] =
          optionalJson( analysis.gates.validation_loss_zero_rad2 );
      gates[ "validation_loss_fit_rad2" ] =
          optionalJson( analysis.gates.validation_loss_fit_rad2 );
      gates[ "blocks_improved" ] =
          optionalJson( analysis.gates.blocks_improved );
      gates[ "blocks_total" ] = optionalJson( analysis.gates.blocks_total );
      gates[ "axis_loss_zero_rad2" ] =
          vectorJson( analysis.gates.axis_loss_zero_rad2 );
      gates[ "axis_loss_fit_rad2" ] =
          vectorJson( analysis.gates.axis_loss_fit_rad2 );
      gates[ "half_bias_max_abs_diff_radps" ] =
          optionalJson( analysis.gates.half_bias_max_abs_diff_radps );
      gates[ "aggregate_reduction" ] =
          optionalJson( analysis.gates.aggregate_reduction );
      gates[ "improved_fraction" ] =
          optionalJson( analysis.gates.improved_fraction );
      gates[ "axis_nonworsening" ] =
          optionalJson( analysis.gates.axis_nonworsening );
      gates[ "half_stability" ] = optionalJson( analysis.gates.half_stability );
      gates[ "all_passed" ]     = optionalJson( analysis.gates.all_passed );
      root[ "gates" ]           = std::move( gates );
      root[ "verdict" ]         = verdictJson( result.verdict );
      return root.dump( 2, ' ', false,
                        nlohmann::ordered_json::error_handler_t::strict ) +
             '\n';
    }

    [[nodiscard]] std::string blocksBytes(
        const GyroAlignmentAnalysis& analysis )
    {
      std::ostringstream stream;
      stream.imbue( std::locale::classic() );
      stream << kBlocksHeader
             << std::setprecision( std::numeric_limits<double>::max_digits10 );
      for ( const auto& block : analysis.blocks )
      {
        stream << "1," << block.block_index << ',' << block.t_start_ns << ','
               << block.t_end_ns << ',' << block.segment_id << ','
               << block.packet_count << ',' << block.r_zero_rad.x() << ','
               << block.r_zero_rad.y() << ',' << block.r_zero_rad.z() << ','
               << block.r_fit_rad.x() << ',' << block.r_fit_rad.y() << ','
               << block.r_fit_rad.z() << ',' << block.loss_zero_rad2 << ','
               << block.loss_fit_rad2 << ',' << ( block.improved ? 1 : 0 )
               << '\n';
      }
      if ( !stream )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                             "failed to serialize blocks CSV" };
      }
      return stream.str();
    }

    struct OutputLedger
    {
      std::string   name;
      std::string   bytes;
      std::uint64_t size{};
      std::string   sha256;
    };

    void writeTempAndRename( const fs::path&     out_dir,
                             const OutputLedger& output )
    {
      const auto    temp  = out_dir / ( "." + output.name + ".tmp" );
      const auto    final = out_dir / output.name;
      std::ofstream stream{ temp, std::ios::binary | std::ios::trunc };
      if ( !stream )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                             "failed to open output temp " + output.name };
      }
      stream.write( output.bytes.data(),
                    static_cast<std::streamsize>( output.bytes.size() ) );
      stream.flush();
      if ( !stream )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                             "failed to flush output temp " + output.name };
      }
      stream.close();
      if ( stream.fail() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                             "failed to close output temp " + output.name };
      }
      std::error_code ec;
      fs::rename( temp, final, ec );
      if ( ec )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputPublish,
                             "failed to publish " + output.name };
      }
    }

    [[nodiscard]] InputBytes readOutput( const fs::path& path,
                                         std::string     name )
    {
      std::ifstream stream{ path, std::ios::binary };
      if ( !stream )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                             "failed to reopen final output " + name };
      }
      std::string bytes{ std::istreambuf_iterator<char>{ stream },
                         std::istreambuf_iterator<char>{} };
      if ( stream.bad() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                             "failed to read final output " + name };
      }
      stream.close();
      if ( stream.fail() )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                             "failed to close final output " + name };
      }
      const auto size   = static_cast<std::uint64_t>( bytes.size() );
      auto       digest = sha256( bytes );
      return InputBytes{ std::move( name ), std::move( bytes ),
                         std::move( digest ), size };
    }

    [[nodiscard]] std::string manifestBytes(
        const GyroAlignmentProtocolDescriptor& descriptor,
        const std::array<InputBytes, 4>&       inputs,
        const GyroAlignmentVerdict&            verdict,
        const std::array<OutputLedger, 2>&     outputs )
    {
      nlohmann::ordered_json root;
      root[ "schema_version" ] = "phad.gyro_alignment.manifest.v1";
      root[ "protocol_id" ]    = descriptor.protocol_id;
      root[ "protocol_identity" ] =
          protocolIdentityJson( descriptor.protocol_identity );
      nlohmann::ordered_json verdict_value;
      verdict_value[ "status" ]    = statusName( verdict.status );
      verdict_value[ "exit_code" ] = verdict.exit_code;
      root[ "verdict" ]            = std::move( verdict_value );
      auto science                 = nlohmann::ordered_json::array();
      for ( const auto& output : outputs )
      {
        nlohmann::ordered_json item;
        item[ "name" ]       = output.name;
        item[ "size_bytes" ] = output.size;
        item[ "sha256" ]     = output.sha256;
        science.push_back( std::move( item ) );
      }
      root[ "science_outputs" ] = std::move( science );
      root[ "inputs" ]          = inputsJson( inputs );
      return root.dump( 2, ' ', false,
                        nlohmann::ordered_json::error_handler_t::strict ) +
             '\n';
    }

    void cleanupOutputs( const fs::path& out_dir ) noexcept
    {
      constexpr std::array<std::string_view, 6> kNames{
          ".gyro_alignment.json.tmp", ".gyro_alignment_blocks.csv.tmp",
          ".gyro_alignment.manifest.json.tmp", "gyro_alignment.json",
          "gyro_alignment_blocks.csv", "gyro_alignment.manifest.json" };
      for ( const auto name : kNames )
      {
        std::error_code ignored;
        fs::remove( out_dir / name, ignored );
      }
    }

    void publish( const fs::path&                        out_dir,
                  const GyroAlignmentProtocolDescriptor& descriptor,
                  const std::array<InputBytes, 4>&       inputs,
                  const GyroAlignmentResult&             result )
    {
      std::error_code ec;
      const bool      created = fs::create_directory( out_dir, ec );
      if ( ( !created && !ec ) || ec == std::errc::file_exists )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputExists,
                             "output path already exists" };
      }
      if ( ec )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                             "failed to create output directory" };
      }
      try
      {
        std::array<OutputLedger, 2> outputs{
            OutputLedger{ "gyro_alignment.json",
                          summaryBytes( descriptor, inputs, result ),
                          0U,
                          {} },
            OutputLedger{ "gyro_alignment_blocks.csv",
                          blocksBytes( *result.analysis ),
                          0U,
                          {} },
        };
        for ( auto& output : outputs )
        {
          output.size   = static_cast<std::uint64_t>( output.bytes.size() );
          output.sha256 = sha256( output.bytes );
          writeTempAndRename( out_dir, output );
        }
        for ( const auto& expected : outputs )
        {
          const auto actual = readOutput( out_dir / expected.name,
                                          expected.name );
          if ( actual.size != expected.size ||
               actual.sha256 != expected.sha256 )
          {
            throw RunnerFailure{
                GyroAlignmentErrorCode::kOutputRehashMismatch,
                "final science output does not match in-memory ledger" };
          }
        }
        OutputLedger manifest{ "gyro_alignment.manifest.json",
                               manifestBytes( descriptor, inputs,
                                              result.verdict, outputs ),
                               0U,
                               {} };
        manifest.size   = static_cast<std::uint64_t>( manifest.bytes.size() );
        manifest.sha256 = sha256( manifest.bytes );
        writeTempAndRename( out_dir, manifest );
      }
      catch ( ... )
      {
        cleanupOutputs( out_dir );
        throw;
      }
    }

    [[nodiscard]] bool outputAlreadyExists( const fs::path& out_dir )
    {
      std::error_code ec;
      const auto      status = fs::symlink_status( out_dir, ec );
      if ( !ec )
      {
        return status.type() != fs::file_type::not_found;
      }
      if ( ec == std::errc::no_such_file_or_directory )
      {
        return false;
      }
      throw RunnerFailure{ GyroAlignmentErrorCode::kOutputIo,
                           "failed to inspect output path" };
    }

  }  // namespace

  GyroAlignmentProtocolDescriptor makeGyroAlignmentV1ProtocolDescriptor()
  {
    std::vector<GyroAlignmentInputIdentity> inputs;
    inputs.reserve( kInputNames.size() );
    for ( std::size_t i = 0; i < kInputNames.size(); ++i )
    {
      inputs.push_back( { std::string{ kInputNames[ i ] },
                          std::string{ kV1Hashes[ i ] } } );
    }
    return GyroAlignmentProtocolDescriptor{
        .protocol_id       = std::string{ kV1ProtocolId },
        .inputs            = std::move( inputs ),
        .protocol_identity = { PHAD_Q3_DESIGN_COMMIT, PHAD_Q3_DESIGN_TREE,
                               PHAD_Q3_DESIGN_BLOB, PHAD_Q3_DESIGN_SHA256 },
        .analyzer_identity = { PHAD_Q3_SOURCE_COMMIT, PHAD_Q3_SOURCE_TREE,
                               PHAD_Q3_BUILD_TYPE, PHAD_Q3_COMPILER,
                               PHAD_Q3_COMPILER_VERSION },
        .provenance        = { "q1_observe_final_q1_final_verifier",
                               "74270572cc1fcc2eac82559117efd0951c800ab9",
                               "e4379bf44db8d1127450d674b60b2e451fbe0aef",
                               "bb5a7854e4a3a89a52c8c0332e965474b8b9b2f2d4cbe0b83bcd7224e35f94ec",
                               "bbeaa21edaee34733f61b8ef093d45d5b8f4268fc4c0a36da261ee6664e9fab1",
                               "402d1925",
                               "aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5",
                               "d1c4385809a6bf461f1c6b8acd81870f98634aa0",
                               "9c9c991b21c4d40e8c5f2ce974334b76e1a42b64" },
    };
  }

  GyroAlignmentResult runGyroAlignment(
      GyroAlignmentProtocolDescriptor descriptor, const fs::path& q1_dir,
      const fs::path& out_dir )
  {
    const bool valid_descriptor =
        descriptor.protocol_id == kV1ProtocolId
            ? sameDescriptor( descriptor,
                              makeGyroAlignmentV1ProtocolDescriptor() )
            : validSyntheticDescriptor( descriptor );
    if ( !valid_descriptor )
    {
      return hardResult( GyroAlignmentErrorCode::kInvalidProtocolDescriptor,
                         "descriptor does not match an allowed protocol binding" );
    }

    try
    {
      const UniqueFd            q1_fd{ openDirectory( q1_dir ) };
      std::array<InputBytes, 4> inputs;
      for ( std::size_t i = 0; i < inputs.size(); ++i )
      {
        inputs[ i ] = readInput( q1_fd.get(),
                                 descriptor.inputs[ i ].basename );
      }
      for ( std::size_t i = 0; i < inputs.size(); ++i )
      {
        inputs[ i ].sha256 = sha256( inputs[ i ].bytes );
        if ( inputs[ i ].sha256 != descriptor.inputs[ i ].expected_sha256 )
        {
          throw RunnerFailure{ GyroAlignmentErrorCode::kInputHashMismatch,
                               "input SHA-256 mismatch for " + inputs[ i ].name };
        }
      }
      if ( outputAlreadyExists( out_dir ) )
      {
        throw RunnerFailure{ GyroAlignmentErrorCode::kOutputExists,
                             "output path already exists" };
      }

      const auto input  = parseAndJoin( inputs );
      auto       result = analyzeGyroAlignment( input );
      if ( result.verdict.status == GyroAlignmentStatus::kHardError )
      {
        if ( !result.error.has_value() )
        {
          return hardResult( GyroAlignmentErrorCode::kNonfiniteComputation,
                             "analyzer returned hard error without code" );
        }
        return hardResult( *result.error, result.verdict.detail );
      }
      if ( result.error.has_value() || !result.analysis.has_value() )
      {
        return hardResult( GyroAlignmentErrorCode::kNonfiniteComputation,
                           "analyzer returned an inconsistent completed result" );
      }
      publish( out_dir, descriptor, inputs, result );
      return result;
    }
    catch ( const RunnerFailure& failure )
    {
      return hardResult( failure.code, failure.detail );
    }
    catch ( const std::exception& error )
    {
      return hardResult( GyroAlignmentErrorCode::kOutputIo, error.what() );
    }
  }

}  // namespace phad::apps
