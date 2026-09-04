#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
#ifdef __linux__
#include <link.h>
#endif
#include <openssl/evp.h>
#include <sys/stat.h>

#include <nlohmann/json.hpp>

#if !defined( PHAD_M5_51_EXPECTED_CXX_COMPILER_PATH ) ||    \
    !defined( PHAD_M5_51_EXPECTED_CXX_COMPILER_VERSION ) || \
    !defined( PHAD_M5_51_EXPECTED_CMAKE_PATH ) ||           \
    !defined( PHAD_M5_51_EXPECTED_CMAKE_VERSION ) ||        \
    !defined( PHAD_M5_51_EXPECTED_BUILD_TOOL_PATH ) ||      \
    !defined( PHAD_M5_51_EXPECTED_GENERATOR ) ||            \
    !defined( PHAD_M5_51_EXPECTED_BUILD_TYPE ) ||           \
    !defined( PHAD_M5_51_EXPECTED_BUILD_TESTS ) ||          \
    !defined( PHAD_M5_51_EXPECTED_TRAJ_EVAL_PATH )
#error "M5 #51 replay diagnostic requires trusted build identity definitions"
#endif
#endif

#include "apps/gyro_observe_writer.hpp"
#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
#include "apps/internal/m5_51_prefix_diagnostic.hpp"
#endif
#if defined( PHAD_M5_51_REPLAY_DIAGNOSTIC_CLI_FIXTURE_TESTING )
#include "tests/apps/m5_51_replay_diagnostic_cli_fixture.hpp"
#endif
#include "apps/internal/vo_bench_dataset.hpp"
#include "apps/offline_vo_session.hpp"
#include "phad/bench/code_identity.hpp"
#include "phad/bench/config_snapshot.hpp"
#include "phad/bench/run_paths.hpp"
#include "phad/bench/run_summary.hpp"
#include "phad/eval/ate.hpp"
#include "phad/eval/error_stats.hpp"
#include "phad/eval/rpe.hpp"
#include "phad/eval/tum_io.hpp"

/**
 * @file phad_vo_bench.cpp
 * @brief VO 回归 composition root：session + eval + bench 落盘。
 */

namespace
{

  using DatasetFormat = phad::apps::internal::VoBenchDatasetFormat;

  constexpr std::string_view kUsage =
      "usage: phad_vo_bench <sequence-root>\n"
      "                     [--bench-root <dir> | --out <dir>]\n"
      "                     [--sequence-name <name>] [--config-label <name>]\n"
      "                     [--dataset-format <euroc|tum-vi>]\n"
      "                     [--gt-euroc <root> | --gt-tum-vi <root>]\n"
      "                     [--max-dt-ms <v>]\n"
      "                     [--min-match-rate <v>] [--rpe-delta-s <v>]\n"
      "                     [--errors-csv] [--gyro-observe] [--repo <dir>]\n"
      "                     [--max-frames <n>] [--force]\n"
      "                     [--no-outlier-cull] [--no-outlier-reopt]\n"
      "                     [--allow-culled-rebirth]\n"
      "                     [--no-drop-culled-tracks]\n"
      "                     [--skip-drop-min-culled <n>]\n"
      "                     [--outlier-avg-reproj-px <v>]\n"
      "                     [--max-outlier-reopts <n>]\n"
      "                     [--probe-b <path>]\n"
      "                     [--defer-drop-topk <k>]\n"
      "                     [--evict-skip-culled]\n"
      "                     [--zombie-drop-age <n>]\n"
      "                     [--hanging-gate-m <meters>]\n"
      "                     [--far-refresh-px <px>]\n"
      "                     [--tracker-enable-census]\n"
      "                     [--tracker-disable-exposure-norm]\n"
      "                     [--estimator-enable-moving-bootstrap]\n"
      "                     [--estimator-enable-pnp-cheirality-retry]\n"
      "                     [--min-seed-observations <n>]\n";

#ifndef PHAD_SOURCE_DIR
#define PHAD_SOURCE_DIR ""
#endif

  struct Arguments
  {
    std::filesystem::path        sequence_root;
    std::filesystem::path        bench_root;
    std::filesystem::path        out_dir;
    std::filesystem::path        gt_euroc;
    std::filesystem::path        gt_tum_vi;
    std::filesystem::path        repo           = PHAD_SOURCE_DIR;
    DatasetFormat                dataset_format = DatasetFormat::kEuroc;
    std::string                  sequence_name;
    std::string                  config_label   = "default";
    double                       max_dt_ms      = 2.5;
    double                       min_match_rate = 0.5;
    double                       rpe_delta_s    = 1.0;
    std::optional<std::uint64_t> max_frames;
    bool                         write_errors_csv      = false;
    bool                         force                 = false;
    bool                         gyro_observe          = false;
    bool                         no_outlier_cull       = false;
    bool                         no_outlier_reopt      = false;
    bool                         allow_culled_rebirth  = false;
    bool                         no_drop_culled_tracks = false;
    std::optional<double>        outlier_avg_reproj_px;
    std::optional<int>           max_outlier_reopts;
    std::optional<int>           skip_drop_min_culled;
    std::filesystem::path        probe_b_path;
    std::optional<int>           defer_drop_topk;
    bool                         evict_skip_culled = false;
    std::optional<int>           zombie_drop_age;
    std::optional<double>        hanging_gate_m;
    std::optional<double>        far_refresh_px;
    // Root visual-seed threshold; enters flattenConfig / config_hash.
    std::optional<int> min_seed_observations;

    // Attribution A/B overrides (Slice ⑥ frontend mechanisms).
    std::optional<double> tracker_quality_level;
    std::optional<int>    tracker_lk_pyramid_levels;
    bool                  no_clahe       = false;
    bool                  no_median_flow = false;
    bool                  no_fransac     = false;
    bool                  no_census      = true;  // census 默认关闭(实测否决)
    // 曝光归一化默认关 (实测否决, 见 stereoTracker.hpp 注释);
    // --tracker-enable-exposure-norm 打开 (A/B)。
    bool enable_exposure_norm = false;
    // 零均值 SAD 默认关 (enable_zero_mean_sad=false), --tracker-enable-
    // zero-mean-sad 打开 (A/B)。
    bool enable_zero_mean_sad = false;
    // Explicit product option; enters flattenConfig / config_hash.
    bool enable_moving_bootstrap = false;
    // Explicit default-off product-candidate option; enters identity only
    // when enabled so the established default hash remains stable.
    bool enable_pnp_cheirality_retry = false;
#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
    std::filesystem::path m5_51_state_receipt;
    std::filesystem::path m5_51_baseline_qualification;
    std::filesystem::path m5_51_comparison_receipt;
    bool                  m5_51_selector_warmup = false;
#endif
  };

  [[nodiscard]] bool parseDouble( std::string_view text, double& value )
  {
    const auto parsed =
        std::from_chars( text.data(), text.data() + text.size(), value );
    return parsed.ec == std::errc{} &&
           parsed.ptr == text.data() + text.size() && std::isfinite( value );
  }

  [[nodiscard]] bool parseUint64( std::string_view text, std::uint64_t& value )
  {
    const auto parsed =
        std::from_chars( text.data(), text.data() + text.size(), value );
    return parsed.ec == std::errc{} &&
           parsed.ptr == text.data() + text.size();
  }

  [[nodiscard]] constexpr bool optionTakesValue( std::string_view option )
  {
    return option == "--bench-root" || option == "--out" ||
           option == "--sequence-name" || option == "--config-label" ||
           option == "--dataset-format" || option == "--gt-euroc" ||
           option == "--gt-tum-vi" || option == "--repo" ||
           option == "--max-dt-ms" || option == "--min-match-rate" ||
           option == "--rpe-delta-s" || option == "--max-frames" ||
           option == "--outlier-avg-reproj-px" ||
           option == "--max-outlier-reopts" ||
           option == "--skip-drop-min-culled" || option == "--probe-b" ||
           option == "--defer-drop-topk" || option == "--zombie-drop-age" ||
           option == "--hanging-gate-m" || option == "--far-refresh-px" ||
           option == "--min-seed-observations" ||
           option == "--tracker-quality-level" ||
           option == "--tracker-lk-pyramid-levels";
  }

  [[nodiscard]] bool parseArguments( int argc, char** argv,
                                     Arguments& arguments )
  {
    if ( argc < 2 )
    {
      return false;
    }
    arguments.sequence_root = argv[ 1 ];
    for ( int index = 2; index < argc; ++index )
    {
      const std::string_view flag{ argv[ index ] };
      if ( flag == "--force" )
      {
        arguments.force = true;
        continue;
      }
      if ( flag == "--errors-csv" )
      {
        arguments.write_errors_csv = true;
        continue;
      }
      if ( flag == "--gyro-observe" )
      {
        arguments.gyro_observe = true;
        continue;
      }
      if ( flag == "--no-outlier-cull" )
      {
        arguments.no_outlier_cull = true;
        continue;
      }
      if ( flag == "--no-outlier-reopt" )
      {
        arguments.no_outlier_reopt = true;
        continue;
      }
      if ( flag == "--allow-culled-rebirth" )
      {
        arguments.allow_culled_rebirth = true;
        continue;
      }
      if ( flag == "--no-drop-culled-tracks" )
      {
        arguments.no_drop_culled_tracks = true;
        continue;
      }
      if ( flag == "--evict-skip-culled" )
      {
        arguments.evict_skip_culled = true;
        continue;
      }
      if ( flag == "--no-clahe" )
      {
        arguments.no_clahe = true;
        continue;
      }
      if ( flag == "--no-median-flow" )
      {
        arguments.no_median_flow = true;
        continue;
      }
      if ( flag == "--no-fransac" )
      {
        arguments.no_fransac = true;
        continue;
      }
      // Census 默认关闭(实测否决, 见 stereoTracker.hpp enable_census 注释);
      // 显式 --tracker-enable-census 打开(对照/复测用)。
      if ( flag == "--tracker-enable-census" )
      {
        arguments.no_census = false;
        continue;
      }
      if ( flag == "--tracker-enable-exposure-norm" )
      {
        arguments.enable_exposure_norm = true;
        continue;
      }
      if ( flag == "--tracker-enable-zero-mean-sad" )
      {
        arguments.enable_zero_mean_sad = true;
        continue;
      }
      if ( flag == "--estimator-enable-moving-bootstrap" )
      {
        arguments.enable_moving_bootstrap = true;
        continue;
      }
      if ( flag == "--estimator-enable-pnp-cheirality-retry" )
      {
        arguments.enable_pnp_cheirality_retry = true;
        continue;
      }
#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
      if ( flag == "--m5-51-selector-warmup" )
      {
        arguments.m5_51_selector_warmup = true;
        continue;
      }
      if ( flag == "--m5-51-state-receipt" )
      {
        if ( index + 1 >= argc )
        {
          std::cerr << "missing value for " << flag << '\n';
          return false;
        }
        arguments.m5_51_state_receipt = argv[ ++index ];
        if ( arguments.m5_51_state_receipt.empty() )
        {
          std::cerr << "--m5-51-state-receipt expects a non-empty path\n";
          return false;
        }
        continue;
      }
      if ( flag == "--m5-51-baseline-qualification" )
      {
        if ( index + 1 >= argc )
        {
          std::cerr << "missing value for " << flag << '\n';
          return false;
        }
        arguments.m5_51_baseline_qualification = argv[ ++index ];
        if ( arguments.m5_51_baseline_qualification.empty() )
        {
          std::cerr
              << "--m5-51-baseline-qualification expects a non-empty path\n";
          return false;
        }
        continue;
      }
      if ( flag == "--m5-51-comparison-receipt" )
      {
        if ( index + 1 >= argc )
        {
          std::cerr << "missing value for " << flag << '\n';
          return false;
        }
        arguments.m5_51_comparison_receipt = argv[ ++index ];
        if ( arguments.m5_51_comparison_receipt.empty() )
        {
          std::cerr << "--m5-51-comparison-receipt expects a non-empty path\n";
          return false;
        }
        continue;
      }
#endif
      if ( !optionTakesValue( flag ) )
      {
        std::cerr << "unknown option: " << flag << '\n';
        return false;
      }
      if ( index + 1 >= argc )
      {
        std::cerr << "missing value for " << flag << '\n';
        return false;
      }
      const std::string_view value{ argv[ index + 1 ] };
      ++index;
      if ( flag == "--bench-root" )
      {
        arguments.bench_root = value;
      }
      else if ( flag == "--out" )
      {
        arguments.out_dir = value;
      }
      else if ( flag == "--sequence-name" )
      {
        arguments.sequence_name = std::string( value );
      }
      else if ( flag == "--config-label" )
      {
        arguments.config_label = std::string( value );
      }
      else if ( flag == "--dataset-format" )
      {
        if ( value == "euroc" )
        {
          arguments.dataset_format = DatasetFormat::kEuroc;
        }
        else if ( value == "tum-vi" )
        {
          arguments.dataset_format = DatasetFormat::kTumVi;
        }
        else
        {
          std::cerr << "--dataset-format expects euroc or tum-vi\n";
          return false;
        }
      }
      else if ( flag == "--gt-euroc" )
      {
        arguments.gt_euroc = value;
      }
      else if ( flag == "--gt-tum-vi" )
      {
        arguments.gt_tum_vi = value;
      }
      else if ( flag == "--repo" )
      {
        arguments.repo = value;
      }
      else if ( flag == "--max-dt-ms" )
      {
        if ( !parseDouble( value, arguments.max_dt_ms ) ||
             arguments.max_dt_ms <= 0.0 )
        {
          std::cerr << "--max-dt-ms expects a positive number\n";
          return false;
        }
      }
      else if ( flag == "--min-match-rate" )
      {
        if ( !parseDouble( value, arguments.min_match_rate ) ||
             arguments.min_match_rate < 0.0 ||
             arguments.min_match_rate > 1.0 )
        {
          std::cerr << "--min-match-rate expects a value in [0, 1]\n";
          return false;
        }
      }
      else if ( flag == "--rpe-delta-s" )
      {
        constexpr double kMinRpeDeltaS =
            static_cast<double>( phad::eval::kDefaultRpeDeltaToleranceNs ) *
            1e-9;
        if ( !parseDouble( value, arguments.rpe_delta_s ) ||
             arguments.rpe_delta_s <= kMinRpeDeltaS )
        {
          std::cerr << "--rpe-delta-s expects a number greater than "
                    << kMinRpeDeltaS << " s\n";
          return false;
        }
      }
      else if ( flag == "--max-frames" )
      {
        std::uint64_t max_frames = 0;
        if ( !parseUint64( value, max_frames ) || max_frames == 0U )
        {
          std::cerr << "--max-frames expects a positive integer\n";
          return false;
        }
        arguments.max_frames = max_frames;
      }
      else if ( flag == "--outlier-avg-reproj-px" )
      {
        double px = 0.0;
        if ( !parseDouble( value, px ) || !( px > 0.0 ) )
        {
          std::cerr
              << "--outlier-avg-reproj-px expects a number greater than 0\n";
          return false;
        }
        arguments.outlier_avg_reproj_px = px;
      }
      else if ( flag == "--max-outlier-reopts" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) ||
             parsed > static_cast<std::uint64_t>(
                          std::numeric_limits<int>::max() ) )
        {
          std::cerr
              << "--max-outlier-reopts expects a non-negative integer\n";
          return false;
        }
        arguments.max_outlier_reopts = static_cast<int>( parsed );
      }
      else if ( flag == "--skip-drop-min-culled" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) ||
             parsed > static_cast<std::uint64_t>(
                          std::numeric_limits<int>::max() ) )
        {
          std::cerr
              << "--skip-drop-min-culled expects a non-negative integer\n";
          return false;
        }
        arguments.skip_drop_min_culled = static_cast<int>( parsed );
      }
      else if ( flag == "--probe-b" )
      {
        arguments.probe_b_path = value;
      }
      else if ( flag == "--defer-drop-topk" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) ||
             parsed > static_cast<std::uint64_t>(
                          std::numeric_limits<int>::max() ) )
        {
          std::cerr
              << "--defer-drop-topk expects a non-negative integer\n";
          return false;
        }
        arguments.defer_drop_topk = static_cast<int>( parsed );
      }
      else if ( flag == "--zombie-drop-age" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) ||
             parsed > static_cast<std::uint64_t>(
                          std::numeric_limits<int>::max() ) )
        {
          std::cerr
              << "--zombie-drop-age expects a non-negative integer\n";
          return false;
        }
        arguments.zombie_drop_age = static_cast<int>( parsed );
      }
      else if ( flag == "--hanging-gate-m" )
      {
        double parsed = 0.0;
        if ( !parseDouble( value, parsed ) || parsed < 0.0 )
        {
          std::cerr
              << "--hanging-gate-m expects a non-negative number (meters)\n";
          return false;
        }
        arguments.hanging_gate_m = parsed;
      }
      else if ( flag == "--far-refresh-px" )
      {
        double parsed = 0.0;
        if ( !parseDouble( value, parsed ) || parsed < 0.0 )
        {
          std::cerr
              << "--far-refresh-px expects a non-negative number (pixels)\n";
          return false;
        }
        arguments.far_refresh_px = parsed;
      }
      else if ( flag == "--min-seed-observations" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) || parsed < 1 )
        {
          std::cerr << "--min-seed-observations expects a positive integer\n";
          return false;
        }
        arguments.min_seed_observations = static_cast<int>( parsed );
      }
      else if ( flag == "--tracker-quality-level" )
      {
        double parsed = 0.0;
        if ( !parseDouble( value, parsed ) || !( parsed > 0.0 ) )
        {
          std::cerr << "--tracker-quality-level expects a positive number\n";
          return false;
        }
        arguments.tracker_quality_level = parsed;
      }
      else if ( flag == "--tracker-lk-pyramid-levels" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) || parsed < 1U )
        {
          std::cerr
              << "--tracker-lk-pyramid-levels expects a positive integer\n";
          return false;
        }
        arguments.tracker_lk_pyramid_levels = static_cast<int>( parsed );
      }
      else
      {
        std::cerr << "unknown option: " << flag << '\n';
        return false;
      }
    }

    if ( arguments.sequence_root.empty() )
    {
      return false;
    }
    if ( arguments.dataset_format == DatasetFormat::kEuroc )
    {
      if ( !arguments.gt_tum_vi.empty() )
      {
        std::cerr << "groundtruth option --gt-tum-vi requires "
                     "--dataset-format tum-vi\n";
        return false;
      }
      if ( arguments.gt_euroc.empty() )
      {
        arguments.gt_euroc = arguments.sequence_root;
      }
    }
    else
    {
      if ( !arguments.gt_euroc.empty() )
      {
        std::cerr << "groundtruth option --gt-euroc is incompatible with "
                     "--dataset-format tum-vi\n";
        return false;
      }
      if ( arguments.gt_tum_vi.empty() )
      {
        std::cerr << "--dataset-format tum-vi requires --gt-tum-vi\n";
        return false;
      }
    }
    if ( arguments.bench_root.empty() )
    {
      if ( const char* env = std::getenv( "PHAD_BENCH_ROOT" );
           env != nullptr && env[ 0 ] != '\0' )
      {
        arguments.bench_root = env;
      }
    }
    if ( arguments.out_dir.empty() && arguments.bench_root.empty() )
    {
      std::cerr << "need --bench-root, PHAD_BENCH_ROOT, or --out\n";
      return false;
    }
    if ( arguments.config_label.empty() )
    {
      std::cerr << "--config-label must be non-empty\n";
      return false;
    }
#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
    if ( arguments.m5_51_selector_warmup &&
         ( arguments.m5_51_state_receipt.empty() ||
           arguments.m5_51_baseline_qualification.empty() ||
           arguments.m5_51_comparison_receipt.empty() ) )
    {
      std::cerr << "--m5-51-selector-warmup requires "
                   "--m5-51-state-receipt and "
                   "--m5-51-baseline-qualification and "
                   "--m5-51-comparison-receipt\n";
      return false;
    }
    if ( !arguments.m5_51_selector_warmup &&
         ( !arguments.m5_51_baseline_qualification.empty() ||
           !arguments.m5_51_comparison_receipt.empty() ) )
    {
      std::cerr << "M5 #51 comparison inputs are valid only with "
                   "--m5-51-selector-warmup\n";
      return false;
    }
#endif
    return true;
  }

  [[nodiscard]] std::string resolveSequenceName( const Arguments& arguments )
  {
    if ( !arguments.sequence_name.empty() )
    {
      return arguments.sequence_name;
    }
    std::error_code             ec;
    const std::filesystem::path canonical =
        std::filesystem::weakly_canonical( arguments.sequence_root, ec );
    const std::filesystem::path root =
        ec ? arguments.sequence_root.lexically_normal() : canonical;
    return root.filename().string();
  }

  [[nodiscard]] std::string utcNowIso8601()
  {
    const std::time_t now = std::time( nullptr );
    std::tm           tm{};
#if defined( _WIN32 )
    gmtime_s( &tm, &now );
#else
    gmtime_r( &now, &tm );
#endif
    char buffer[ 32 ];
    if ( std::strftime( buffer, sizeof( buffer ), "%Y-%m-%dT%H:%M:%SZ",
                        &tm ) == 0U )
    {
      return "unknown";
    }
    return buffer;
  }

  [[nodiscard]] bool writeTextFile( const std::filesystem::path& path,
                                    std::string_view             text )
  {
    std::ofstream out( path, std::ios::trunc );
    if ( !out )
    {
      return false;
    }
    out << text;
    out.flush();
    return static_cast<bool>( out );
  }

#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
  [[nodiscard]] std::string readFileBytes(
      const std::filesystem::path& path )
  {
    std::ifstream input( path, std::ios::binary );
    if ( !input )
    {
      throw std::runtime_error( "cannot read " + path.string() );
    }
    return { std::istreambuf_iterator<char>( input ),
             std::istreambuf_iterator<char>() };
  }

  [[nodiscard]] std::string fixedHex( std::uint64_t value,
                                      std::size_t   digits )
  {
    constexpr char kHex[] = "0123456789abcdef";
    std::string    encoded( digits, '0' );
    for ( std::size_t index = 0U; index < digits; ++index )
    {
      encoded[ digits - index - 1U ] = kHex[ value & 0x0fU ];
      value >>= 4U;
    }
    return encoded;
  }

  [[nodiscard]] std::string bytesSha256( const std::string_view bytes )
  {
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    std::size_t                                digest_size = digest.size();
    if ( EVP_Q_digest( nullptr, "SHA256", nullptr, bytes.data(),
                       bytes.size(), digest.data(), &digest_size ) != 1 ||
         digest_size != 32U )
    {
      throw std::runtime_error( "cannot compute SHA-256" );
    }
    std::string encoded;
    encoded.reserve( digest_size * 2U );
    for ( std::size_t index = 0U; index < digest_size; ++index )
    {
      encoded += fixedHex( digest[ index ], 2U );
    }
    return encoded;
  }

  [[nodiscard]] std::string fileSha256(
      const std::filesystem::path& path )
  {
    return bytesSha256( readFileBytes( path ) );
  }

  using RuntimeLibraryIdentities = std::map<std::string, std::string>;

  [[nodiscard]] std::optional<std::string> loadedRuntimeLibraryIdentities(
      RuntimeLibraryIdentities& identities )
  {
#ifdef __linux__
    struct LoadedObjects
    {
      std::set<std::filesystem::path> paths;
      std::string                     error;
    } loaded;

    const auto visit = []( struct dl_phdr_info* info, std::size_t,
                           void*                opaque ) {
      auto& objects = *static_cast<LoadedObjects*>( opaque );
      if ( info == nullptr || info->dlpi_name == nullptr ||
           info->dlpi_name[ 0 ] == '\0' )
      {
        return 0;
      }
      const std::string_view name{ info->dlpi_name };
      if ( name.starts_with( "linux-vdso" ) )
      {
        return 0;
      }

      std::error_code error;
      const auto      canonical =
          std::filesystem::canonical( std::filesystem::path{ name }, error );
      if ( error || !std::filesystem::is_regular_file( canonical ) )
      {
        objects.error = "cannot identify loaded runtime object " +
                        std::string{ name };
        return 1;
      }
      objects.paths.insert( canonical );
      return 0;
    };

    static_cast<void>( ::dl_iterate_phdr( visit, &loaded ) );
    if ( !loaded.error.empty() )
    {
      return loaded.error;
    }
    for ( const auto& path : loaded.paths )
    {
      identities.emplace( path.string(), fileSha256( path ) );
    }
    return std::nullopt;
#else
    static_cast<void>( identities );
    return "loaded runtime identity is supported only on Linux";
#endif
  }

  [[nodiscard]] bool isSafeRelativePath( const std::filesystem::path& path )
  {
    return !path.empty() && !path.is_absolute() &&
           std::find( path.begin(), path.end(), ".." ) == path.end() &&
           path.lexically_normal() == path;
  }

  [[nodiscard]] std::string sourceMode( const std::filesystem::path& path,
                                        struct stat&                 metadata )
  {
    if ( ::lstat( path.c_str(), &metadata ) != 0 )
    {
      throw std::runtime_error( "cannot lstat source entry " + path.string() );
    }
    std::ostringstream encoded;
    encoded << std::oct << std::setw( 4 ) << std::setfill( '0' )
            << ( metadata.st_mode & 07777 );
    return encoded.str();
  }

  [[nodiscard]] std::optional<std::string> verifySourceManifest(
      const std::filesystem::path& repo, const nlohmann::json& document,
      std::string_view expected_source_id )
  {
    if ( document.at( "schema_version" ) != 1 )
    {
      return "source manifest schema mismatch";
    }

    nlohmann::json    canonical = document;
    const std::string declared_source_id =
        canonical.at( "source_id" ).get<std::string>();
    canonical.erase( "source_id" );
    const std::string computed_source_id =
        "sha256:" + bytesSha256( canonical.dump() );
    if ( declared_source_id != computed_source_id ||
         declared_source_id != expected_source_id )
    {
      return "source manifest canonical id mismatch";
    }

    std::set<std::filesystem::path> seen;
    std::string                     previous_path;
    for ( const auto& entry : document.at( "entries" ) )
    {
      const std::string           relative_text = entry.at( "path" ).get<std::string>();
      const std::filesystem::path relative{ relative_text };
      if ( !isSafeRelativePath( relative ) ||
           ( !previous_path.empty() && relative_text <= previous_path ) ||
           !seen.insert( relative ).second )
      {
        return "source manifest entry path is unsafe, duplicated, or unsorted";
      }
      previous_path = relative_text;

      const std::filesystem::path actual = repo / relative;
      struct stat metadata
      {
      };
      const std::string actual_mode = sourceMode( actual, metadata );
      if ( actual_mode != entry.at( "mode" ).get<std::string>() )
      {
        return "current source mode differs at " + relative_text;
      }

      const std::string type = entry.at( "type" ).get<std::string>();
      if ( type == "file" )
      {
        if ( !S_ISREG( metadata.st_mode ) ||
             metadata.st_size < 0 ||
             static_cast<std::uint64_t>( metadata.st_size ) !=
                 entry.at( "size" ).get<std::uint64_t>() ||
             fileSha256( actual ) != entry.at( "sha256" ).get<std::string>() )
        {
          return "current source file differs at " + relative_text;
        }
      }
      else if ( type == "symlink" )
      {
        if ( !S_ISLNK( metadata.st_mode ) )
        {
          return "current source type differs at " + relative_text;
        }
        const std::string target = std::filesystem::read_symlink( actual ).string();
        if ( target != entry.at( "target" ).get<std::string>() ||
             bytesSha256( target ) != entry.at( "sha256" ).get<std::string>() )
        {
          return "current source symlink differs at " + relative_text;
        }
      }
      else
      {
        return "source manifest entry type is unsupported";
      }
    }

    std::set<std::filesystem::path> deleted;
    std::string                     previous_deleted;
    for ( const auto& value : document.at( "deleted_tracked" ) )
    {
      const std::string           relative_text = value.get<std::string>();
      const std::filesystem::path relative{ relative_text };
      if ( !isSafeRelativePath( relative ) ||
           ( !previous_deleted.empty() && relative_text <= previous_deleted ) ||
           !deleted.insert( relative ).second ||
           std::filesystem::exists( repo / relative ) ||
           std::filesystem::is_symlink( repo / relative ) )
      {
        return "source manifest tracked deletion differs at " + relative_text;
      }
      previous_deleted = relative_text;
    }
    return std::nullopt;
  }

  [[nodiscard]] std::optional<std::string> verifyCanonicalState(
      const nlohmann::ordered_json& state, std::string_view field )
  {
    const auto& canonical_json = state.at( "canonical_json" );
    if ( !canonical_json.is_object() ||
         canonical_json.at( "schema" ) !=
             "phad.m5_51.future_runtime_state.v1" ||
         bytesSha256( canonical_json.dump() ) !=
             state.at( "sha256" ).get<std::string>() )
    {
      return std::string{ field } + " canonical state digest mismatch";
    }
    return std::nullopt;
  }

  [[nodiscard]] std::optional<std::string> verifyBaselineStateReceipt(
      const std::filesystem::path& path, std::string_view expected_file_sha )
  {
    const std::string bytes = readFileBytes( path );
    if ( bytesSha256( bytes ) != expected_file_sha )
    {
      return "baseline state receipt file digest mismatch";
    }
    const nlohmann::ordered_json receipt =
        nlohmann::ordered_json::parse( bytes );
    if ( receipt.at( "schema" ) != "m5_51_prefix_diagnostic_receipt_v1" ||
         receipt.at( "hook_frame_index" ) != 6U ||
         !receipt.at( "hook_executed" ).get<bool>() ||
         !receipt.at( "failure_reason" ).get<std::string>().empty() ||
         !receipt.at( "hook_diff" ).empty() ||
         receipt.at( "pre_hook_state_v1" ) !=
             receipt.at( "post_hook_state_v1" ) )
    {
      return "baseline state receipt contract mismatch";
    }
    for ( const std::string_view field :
          { "pre_hook_state_v1", "post_hook_state_v1", "post_state_v1" } )
    {
      if ( const auto error = verifyCanonicalState( receipt.at( field ), field ) )
      {
        return error;
      }
    }
    const auto& diag = receipt.at( "post_state_v1" )
                           .at( "canonical_json" )
                           .at( "session" )
                           .at( "result" )
                           .at( "diag" );
    if ( diag.size() != 43U || !diag.at( 7 ).at( "is_keyframe" ).get<bool>() )
    {
      return "baseline state receipt i=7 lifecycle mismatch";
    }
    return std::nullopt;
  }

  [[nodiscard]] std::optional<std::uint64_t> firstOkIndex(
      const std::filesystem::path& diag_path )
  {
    std::ifstream input( diag_path );
    std::string   line;
    if ( !input || !std::getline( input, line ) )
    {
      return std::nullopt;
    }
    std::uint64_t index = 0U;
    while ( std::getline( input, line ) )
    {
      const std::size_t first_comma = line.find( ',' );
      const std::size_t second_comma =
          first_comma == std::string::npos
              ? std::string::npos
              : line.find( ',', first_comma + 1U );
      if ( first_comma == std::string::npos ||
           second_comma == std::string::npos )
      {
        throw std::runtime_error( "invalid diag.csv row" );
      }
      if ( line.substr( first_comma + 1U,
                        second_comma - first_comma - 1U ) == "ok" )
      {
        return index;
      }
      ++index;
    }
    return std::nullopt;
  }

  [[nodiscard]] std::optional<std::string> verifyInputManifest(
      const std::filesystem::path& sequence_root,
      const std::filesystem::path& manifest_path,
      const std::size_t            expected_entry_count )
  {
    std::ifstream input( manifest_path );
    if ( !input )
    {
      return "cannot read frozen input manifest";
    }

    std::set<std::filesystem::path> expected_paths;
    std::string                     line;
    while ( std::getline( input, line ) )
    {
      const std::size_t separator = line.find( "  " );
      if ( separator != 64U || line.size() <= separator + 2U )
      {
        return "frozen input manifest row is malformed";
      }
      const std::string expected_sha  = line.substr( 0U, separator );
      const std::string relative_text = line.substr( separator + 2U );
      if ( !relative_text.starts_with( "./" ) )
      {
        return "frozen input manifest path is not relative";
      }
      const std::filesystem::path relative =
          std::filesystem::path{ relative_text.substr( 2U ) }.lexically_normal();
      if ( relative.empty() || relative.is_absolute() ||
           std::find( relative.begin(), relative.end(), ".." ) !=
               relative.end() ||
           !expected_paths.insert( relative ).second )
      {
        return "frozen input manifest path is unsafe or duplicated";
      }
      const std::filesystem::path actual = sequence_root / relative;
      if ( !std::filesystem::is_regular_file( actual ) ||
           fileSha256( actual ) != expected_sha )
      {
        return "current sequence content differs at " + relative.string();
      }
    }

    if ( expected_paths.size() != expected_entry_count )
    {
      return "frozen input manifest entry count mismatch";
    }

    std::set<std::filesystem::path> actual_paths;
    for ( const auto& entry :
          std::filesystem::recursive_directory_iterator( sequence_root ) )
    {
      if ( entry.is_regular_file() )
      {
        actual_paths.insert(
            std::filesystem::relative( entry.path(), sequence_root ) );
      }
    }
    if ( actual_paths != expected_paths )
    {
      return "current sequence file set differs from frozen input manifest";
    }
    return std::nullopt;
  }

  struct M5BaselineQualification
  {
    nlohmann::json m_document;
    std::string    m_sha256;
  };

  [[nodiscard]] std::optional<std::string> validateM5BaselineQualification(
      const Arguments& arguments, std::string_view config_hash,
      std::string_view         canonical_config_text,
      M5BaselineQualification& validated_qualification )
  {
    constexpr std::string_view kFormalConfigHash = "0337287b";
#if defined( PHAD_M5_51_REPLAY_DIAGNOSTIC_CLI_FIXTURE_TESTING )
    constexpr std::string_view kInputManifestSha =
        "2b982fd1a2c814d077e0ee4361ca45d8b56ca23aabc65c5c14509d40f4763ea4";
    constexpr std::string_view kDiagSha =
        "ef783a731d0946936e0640065a215138471b58d81f0b55d156ed61228331674d";
    constexpr std::string_view kEstSha =
        "ff390472ab5a66e5d4db829511c305d4155110cb8516e0a32bf4c191770527fe";
    constexpr std::string_view kKeyframeSha =
        "cfa32b9311b7ca9f4e56b1fdc88d32287e0d655d73cb34adcddf7bac4bb195aa";
#else
    constexpr std::string_view kInputManifestSha =
        "edd7053e1152e1cd167cec3564e694696d7d5f25fddb0f4f90b1453a310992db";
    constexpr std::string_view kDiagSha =
        "dde502ec23ef8401d30024aa02a14e1854e5c05d1db054d9a3d82bd307696f7c";
    constexpr std::string_view kEstSha =
        "1f032910d0e2c4464c30d861d2d18d73439a904bbead148c09627c24d35b16b6";
    constexpr std::string_view kKeyframeSha =
        "a5f9c6a423d6fc854cebe98a57d773faa1edbb7fee9fd15199dd941b13adcb7c";
#endif

    try
    {
      const std::string qualification_bytes =
          readFileBytes( arguments.m5_51_baseline_qualification );
      const nlohmann::json qualification =
          nlohmann::json::parse( qualification_bytes );
      if ( qualification.at( "schema" ) !=
               "phad.m5_51.baseline_qualification.v1" ||
           qualification.at( "status" ) != "qualified" )
      {
        return "baseline qualification schema/status mismatch";
      }
      if ( config_hash != kFormalConfigHash ||
           qualification.at( "config_hash" ).get<std::string>() !=
               config_hash )
      {
        return "baseline qualification canonical config mismatch";
      }
      const std::string canonical_config_sha =
          bytesSha256( canonical_config_text );
      if ( !qualification.contains( "config_canonical_text_sha256" ) ||
           !qualification.at( "config_canonical_text_sha256" ).is_string() ||
           qualification.at( "config_canonical_text_sha256" )
                   .get<std::string>() != canonical_config_sha )
      {
        return "baseline qualification canonical config SHA-256 mismatch";
      }

      if ( !qualification.contains( "preflight_identity_path" ) ||
           !qualification.at( "preflight_identity_path" ).is_string() ||
           !qualification.contains( "preflight_identity_sha256" ) ||
           !qualification.at( "preflight_identity_sha256" ).is_string() )
      {
        return "baseline qualification preflight identity missing";
      }
      const std::filesystem::path preflight_path =
          qualification.at( "preflight_identity_path" ).get<std::string>();
      if ( !preflight_path.is_absolute() ||
           !std::filesystem::is_regular_file( preflight_path ) )
      {
        return "baseline qualification preflight identity unreadable";
      }
      const std::string preflight_bytes = readFileBytes( preflight_path );
      if ( bytesSha256( preflight_bytes ) !=
           qualification.at( "preflight_identity_sha256" )
               .get<std::string>() )
      {
        return "baseline qualification preflight identity digest mismatch";
      }
      const nlohmann::json preflight =
          nlohmann::json::parse( preflight_bytes );
      if ( !preflight.contains( "schema" ) ||
           preflight.at( "schema" ) !=
               "phad.m5_51.preflight_identity.v1" )
      {
        return "baseline qualification preflight identity schema mismatch";
      }
      if ( !preflight.contains( "configuration" ) ||
           !preflight.at( "configuration" ).is_object() ||
           !preflight.at( "configuration" )
                .contains( "canonical_text_sha256" ) ||
           !preflight.at( "configuration" )
                .at( "canonical_text_sha256" )
                .is_string() ||
           preflight.at( "configuration" )
                   .at( "canonical_text_sha256" )
                   .get<std::string>() != canonical_config_sha ||
           preflight.at( "configuration" )
                   .at( "canonical_text_sha256" )
                   .get<std::string>() !=
               qualification.at( "config_canonical_text_sha256" )
                   .get<std::string>() )
      {
        return "baseline qualification preflight identity canonical config "
               "SHA-256 mismatch";
      }

      const auto isSha256 = []( const std::string_view value ) {
        return value.size() == 64U &&
               std::all_of( value.begin(), value.end(), []( const char digit ) {
                 return ( digit >= '0' && digit <= '9' ) ||
                        ( digit >= 'a' && digit <= 'f' );
               } );
      };
      const auto hasFileIdentity = [ &isSha256 ](
                                       const nlohmann::json& identity ) {
        if ( !identity.is_object() || !identity.contains( "path" ) ||
             !identity.at( "path" ).is_string() ||
             !identity.contains( "sha256" ) ||
             !identity.at( "sha256" ).is_string() )
        {
          return false;
        }
        const std::filesystem::path path =
            identity.at( "path" ).get<std::string>();
        const std::string sha = identity.at( "sha256" ).get<std::string>();
        return path.is_absolute() && std::filesystem::is_regular_file( path ) &&
               isSha256( sha ) && fileSha256( path ) == sha;
      };
      const auto hasExpectedFileIdentity =
          [ &hasFileIdentity ]( const nlohmann::json&        identity,
                                const std::filesystem::path& expected ) {
            return hasFileIdentity( identity ) && expected.is_absolute() &&
                   std::filesystem::is_regular_file( expected ) &&
                   std::filesystem::canonical(
                       identity.at( "path" ).get<std::string>() ) ==
                       std::filesystem::canonical( expected );
          };

      const nlohmann::json& preflight_source = preflight.at( "source" );
      if ( preflight_source.at( "source_id" ) !=
               qualification.at( "source_id" ) ||
           preflight_source.at( "manifest_path" ) !=
               qualification.at( "source_manifest_path" ) ||
           preflight_source.at( "manifest_sha256" ) !=
               qualification.at( "source_manifest_sha256" ) ||
           preflight_source.at( "archive_path" ) !=
               qualification.at( "source_archive_path" ) ||
           preflight_source.at( "archive_sha256" ) !=
               qualification.at( "source_archive_sha256" ) ||
           preflight_source.at( "base_bundle_path" ) !=
               qualification.at( "base_bundle_path" ) ||
           preflight_source.at( "base_bundle_sha256" ) !=
               qualification.at( "base_bundle_sha256" ) ||
           preflight_source.at( "verification_receipt_path" ) !=
               qualification.at( "source_verification_receipt_path" ) ||
           preflight_source.at( "verification_receipt_sha256" ) !=
               qualification.at( "source_verification_receipt_sha256" ) )
      {
        return "baseline qualification preflight source identity mismatch";
      }

      const std::filesystem::path executable =
          std::filesystem::canonical( "/proc/self/exe" );
      const std::string     executable_sha = fileSha256( executable );
      const nlohmann::json& preflight_binary =
          preflight.at( "binaries" ).at( "phad_vo_bench" );
      if ( !hasFileIdentity( preflight_binary ) ||
           std::filesystem::canonical(
               preflight_binary.at( "path" ).get<std::string>() ) !=
               executable ||
           preflight_binary.at( "sha256" ).get<std::string>() !=
               executable_sha ||
           qualification.at( "binary_sha256" ).get<std::string>() !=
               executable_sha )
      {
        return "baseline qualification preflight binary identity mismatch";
      }

      const nlohmann::json& preflight_input = preflight.at( "input" );
      const auto            input_entry_count =
          qualification.at( "input_entry_count" ).get<std::size_t>();
      if ( preflight_input.at( "ordered_manifest_path" ) !=
               qualification.at( "input_manifest_path" ) ||
           preflight_input.at( "ordered_manifest_sha256" ) !=
               qualification.at( "input_manifest_sha256" ) ||
           preflight_input.at( "entry_count" ).get<std::size_t>() !=
               input_entry_count ||
           preflight_input.at( "verified_entry_count" ).get<std::size_t>() !=
               input_entry_count ||
           std::filesystem::path{
               preflight_input.at( "root" ).get<std::string>() }
                   .lexically_normal() !=
               arguments.sequence_root.lexically_normal() ||
           preflight_input.at( "sequence" ).get<std::string>() !=
               resolveSequenceName( arguments ) )
      {
        return "baseline qualification preflight input identity mismatch";
      }

      const nlohmann::json& preflight_evaluator =
          preflight.at( "evaluator" );
      const std::filesystem::path bench_source =
          arguments.repo / "apps/phad_vo_bench.cpp";
      const std::filesystem::path rpe_cpp = arguments.repo / "phad/eval/rpe.cpp";
      const std::filesystem::path rpe_hpp = arguments.repo / "phad/eval/rpe.hpp";
      const nlohmann::json&       traj_eval =
          preflight.at( "binaries" ).at( "phad_traj_eval" );
      if ( !std::filesystem::is_regular_file( bench_source ) ||
           !std::filesystem::is_regular_file( rpe_cpp ) ||
           !std::filesystem::is_regular_file( rpe_hpp ) ||
           preflight_evaluator.at( "bench_source_sha256" ) !=
               fileSha256( bench_source ) ||
           preflight_evaluator.at( "rpe_cpp_sha256" ) !=
               fileSha256( rpe_cpp ) ||
           preflight_evaluator.at( "rpe_cpp_sha256" ) !=
               qualification.at( "rpe_evaluator_sha256" ) ||
           preflight_evaluator.at( "rpe_hpp_sha256" ) !=
               fileSha256( rpe_hpp ) ||
           preflight_evaluator.at( "max_dt_ms" ).get<double>() !=
               arguments.max_dt_ms ||
           preflight_evaluator.at( "min_match_rate" ).get<double>() !=
               arguments.min_match_rate ||
           preflight_evaluator.at( "rpe_delta_s" ).get<double>() !=
               arguments.rpe_delta_s ||
           !hasExpectedFileIdentity(
               traj_eval,
               std::filesystem::path{ PHAD_M5_51_EXPECTED_TRAJ_EVAL_PATH } ) )
      {
        return "baseline qualification preflight evaluator identity mismatch";
      }

      const nlohmann::json& build = preflight.at( "build" );
      if ( build.at( "type" ) != PHAD_M5_51_EXPECTED_BUILD_TYPE ||
           build.at( "generator" ) != PHAD_M5_51_EXPECTED_GENERATOR ||
           !build.at( "phad_build_tests" ).is_boolean() ||
           build.at( "phad_build_tests" ).get<bool>() !=
               static_cast<bool>( PHAD_M5_51_EXPECTED_BUILD_TESTS ) )
      {
        return "baseline qualification preflight build identity mismatch";
      }

      const nlohmann::json& compiler = build.at( "compiler" );
      if ( compiler.at( "version" ) !=
               PHAD_M5_51_EXPECTED_CXX_COMPILER_VERSION ||
           !hasExpectedFileIdentity(
               compiler,
               std::filesystem::path{
                   PHAD_M5_51_EXPECTED_CXX_COMPILER_PATH } ) )
      {
        return "baseline qualification preflight compiler identity mismatch";
      }
      const nlohmann::json& cmake = build.at( "cmake" );
      if ( cmake.at( "version" ) != PHAD_M5_51_EXPECTED_CMAKE_VERSION ||
           !hasExpectedFileIdentity(
               cmake,
               std::filesystem::path{ PHAD_M5_51_EXPECTED_CMAKE_PATH } ) )
      {
        return "baseline qualification preflight cmake identity mismatch";
      }
      const nlohmann::json& build_tool = build.at( "ninja" );
      if ( !build_tool.contains( "version" ) ||
           !build_tool.at( "version" ).is_string() ||
           build_tool.at( "version" ).get<std::string>().empty() ||
           !hasExpectedFileIdentity(
               build_tool,
               std::filesystem::path{
                   PHAD_M5_51_EXPECTED_BUILD_TOOL_PATH } ) )
      {
        return "baseline qualification preflight build tool identity mismatch";
      }

      const nlohmann::json& preflight_config =
          preflight.at( "configuration" );
      const std::filesystem::path reference_meta =
          preflight_config.at( "reference_meta_path" ).get<std::string>();
      if ( preflight_config.at( "label" ) != arguments.config_label ||
           preflight_config.at( "canonical_hash" ) != config_hash ||
           preflight_config.at( "canonical_text_sha256" ) !=
               canonical_config_sha ||
           preflight_config.at( "moving_bootstrap" ).get<bool>() !=
               arguments.enable_moving_bootstrap ||
           preflight_config.at( "pnp_cheirality_retry" ).get<bool>() !=
               arguments.enable_pnp_cheirality_retry ||
           preflight_config.at( "collect_timing" ).get<bool>() ||
           !arguments.max_frames.has_value() ||
           preflight_config.at( "execution_bound_max_frames" ) !=
               *arguments.max_frames ||
           !std::filesystem::is_regular_file( reference_meta ) ||
           fileSha256( reference_meta ) !=
               preflight_config.at( "reference_meta_sha256" )
                   .get<std::string>() )
      {
        return "baseline qualification preflight configuration identity "
               "mismatch";
      }

      const nlohmann::json& runtime = preflight.at( "runtime" );
      for ( const std::string_view version :
            { "opencv_version", "gtsam_version", "openssl_version" } )
      {
        if ( !runtime.at( version ).is_string() ||
             runtime.at( version ).get<std::string>().empty() )
        {
          return "baseline qualification preflight runtime identity mismatch";
        }
      }
      const nlohmann::json& determinism =
          runtime.at( "determinism_environment" );
      for ( const char* variable :
            { "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS" } )
      {
        const char* value = std::getenv( variable );
        if ( value == nullptr || !determinism.contains( variable ) ||
             !determinism.at( variable ).is_string() ||
             determinism.at( variable ).get<std::string>() != value )
        {
          return "baseline qualification preflight runtime identity mismatch";
        }
      }

      const std::filesystem::path dynamic_libraries =
          runtime.at( "dynamic_libraries_path" ).get<std::string>();
      if ( !std::filesystem::is_regular_file( dynamic_libraries ) ||
           fileSha256( dynamic_libraries ) !=
               runtime.at( "dynamic_libraries_sha256" ).get<std::string>() )
      {
        return "baseline qualification preflight runtime library identity "
               "mismatch";
      }
      std::ifstream            dynamic_input( dynamic_libraries );
      RuntimeLibraryIdentities expected_runtime_libraries;
      std::string              dynamic_line;
      while ( std::getline( dynamic_input, dynamic_line ) )
      {
        if ( dynamic_line.size() <= 66U || dynamic_line[ 64U ] != ' ' ||
             dynamic_line[ 65U ] != ' ' )
        {
          return "baseline qualification preflight runtime library identity "
                 "mismatch";
        }
        const std::string           expected_sha = dynamic_line.substr( 0U, 64U );
        const std::filesystem::path library{ dynamic_line.substr( 66U ) };
        std::error_code             canonical_error;
        const auto                  canonical_library =
            std::filesystem::canonical( library, canonical_error );
        if ( !isSha256( expected_sha ) || !library.is_absolute() ||
             canonical_error ||
             !std::filesystem::is_regular_file( canonical_library ) ||
             fileSha256( canonical_library ) != expected_sha ||
             !expected_runtime_libraries
                  .emplace( canonical_library.string(), expected_sha )
                  .second )
        {
          return "baseline qualification preflight runtime library identity "
                 "mismatch";
        }
      }
      if ( !dynamic_input.eof() ||
           expected_runtime_libraries.size() !=
               runtime.at( "dynamic_library_count" ).get<std::size_t>() )
      {
        return "baseline qualification preflight runtime library identity "
               "mismatch";
      }
      RuntimeLibraryIdentities loaded_runtime_libraries;
      if ( const auto loaded_error =
               loadedRuntimeLibraryIdentities( loaded_runtime_libraries ) )
      {
        return "baseline qualification preflight runtime library identity "
               "mismatch: " +
               *loaded_error;
      }
      if ( loaded_runtime_libraries != expected_runtime_libraries )
      {
        return "baseline qualification preflight runtime library identity "
               "loaded set mismatch";
      }

      const std::filesystem::path source_manifest =
          qualification.at( "source_manifest_path" ).get<std::string>();
      const std::string source_manifest_sha = fileSha256( source_manifest );
      if ( source_manifest_sha !=
           qualification.at( "source_manifest_sha256" ).get<std::string>() )
      {
        return "baseline qualification source manifest digest mismatch";
      }
      const nlohmann::json source =
          nlohmann::json::parse( readFileBytes( source_manifest ) );
      const std::string source_id =
          qualification.at( "source_id" ).get<std::string>();
      if ( preflight_source.at( "entry_count" ).get<std::size_t>() !=
               source.at( "entries" ).size() ||
           preflight_source.at( "deleted_tracked_count" )
                   .get<std::size_t>() !=
               source.at( "deleted_tracked" ).size() ||
           preflight_source.at( "git_head" ) !=
               source.at( "git" ).at( "head" ) ||
           preflight_source.at( "git_tree" ) !=
               source.at( "git" ).at( "tree" ) )
      {
        return "baseline qualification preflight source identity mismatch";
      }
      if ( const auto source_error =
               verifySourceManifest( arguments.repo, source, source_id ) )
      {
        return *source_error;
      }

      const std::filesystem::path source_archive =
          qualification.at( "source_archive_path" ).get<std::string>();
      const std::filesystem::path base_bundle =
          qualification.at( "base_bundle_path" ).get<std::string>();
      const std::string source_archive_sha = fileSha256( source_archive );
      const std::string base_bundle_sha    = fileSha256( base_bundle );
      if ( source_archive_sha !=
               qualification.at( "source_archive_sha256" ).get<std::string>() ||
           base_bundle_sha !=
               qualification.at( "base_bundle_sha256" ).get<std::string>() )
      {
        return "baseline qualification source transport hash mismatch";
      }

      const std::filesystem::path verification_path =
          qualification.at( "source_verification_receipt_path" )
              .get<std::string>();
      if ( fileSha256( verification_path ) !=
           qualification.at( "source_verification_receipt_sha256" )
               .get<std::string>() )
      {
        return "baseline qualification source verification digest mismatch";
      }
      const nlohmann::json verification =
          nlohmann::json::parse( readFileBytes( verification_path ) );
      const nlohmann::json& verified = verification.at( "verified" );
      if ( verification.at( "schema" ) !=
               "phad.m5_51.source_verification.v1" ||
           verification.at( "source_id" ) != source_id ||
           verification.at( "source_manifest_sha256" ) !=
               source_manifest_sha ||
           verification.at( "source_archive_sha256" ) != source_archive_sha ||
           verification.at( "base_bundle_sha256" ) != base_bundle_sha ||
           std::filesystem::canonical(
               verification.at( "source_root" ).get<std::string>() ) !=
               std::filesystem::canonical( arguments.repo ) ||
           verified.at( "schema_version" ) != source.at( "schema_version" ) ||
           verified.at( "source_id" ) != source_id ||
           verified.at( "git_head" ) != source.at( "git" ).at( "head" ) ||
           verified.at( "git_tree" ) != source.at( "git" ).at( "tree" ) ||
           verified.at( "git_branch" ) != source.at( "git" ).at( "branch" ) ||
           verified.at( "entry_count" ).get<std::size_t>() !=
               source.at( "entries" ).size() ||
           verified.at( "deleted_tracked_count" ).get<std::size_t>() !=
               source.at( "deleted_tracked" ).size() )
      {
        return "baseline qualification source verification mismatch";
      }

      if ( executable_sha !=
           qualification.at( "binary_sha256" ).get<std::string>() )
      {
        return "baseline qualification binary mismatch";
      }

      const std::filesystem::path input_manifest =
          qualification.at( "input_manifest_path" ).get<std::string>();
      if ( fileSha256( input_manifest ) != kInputManifestSha ||
           qualification.at( "input_manifest_sha256" ).get<std::string>() !=
               kInputManifestSha )
      {
        return "baseline qualification input manifest mismatch";
      }
      if ( const auto input_error =
               verifyInputManifest( arguments.sequence_root, input_manifest,
                                    input_entry_count ) )
      {
        return *input_error;
      }

      const std::filesystem::path evaluator =
          arguments.repo / "phad/eval/rpe.cpp";
      if ( fileSha256( evaluator ) !=
           qualification.at( "rpe_evaluator_sha256" ).get<std::string>() )
      {
        return "baseline qualification RPE evaluator mismatch";
      }

      const std::filesystem::path artifact_dir =
          qualification.at( "baseline_artifact_dir" ).get<std::string>();
      if ( fileSha256( artifact_dir / "diag.csv" ) != kDiagSha ||
           fileSha256( artifact_dir / "est.tum" ) != kEstSha ||
           fileSha256( artifact_dir / "kf.tum" ) != kKeyframeSha )
      {
        return "baseline qualification prefix oracle mismatch";
      }
      const auto first_ok = firstOkIndex( artifact_dir / "diag.csv" );
      if ( !first_ok.has_value() || *first_ok != 6U )
      {
        return "baseline qualification first-ok mismatch";
      }
      const nlohmann::json meta = nlohmann::json::parse(
          readFileBytes( artifact_dir / "meta.json" ) );
      if ( meta.at( "config_hash" ) != config_hash ||
           meta.at( "sequence_root" ) != arguments.sequence_root.string() )
      {
        return "baseline qualification run identity mismatch";
      }

      const std::filesystem::path state_receipt =
          qualification.at( "baseline_state_receipt" ).get<std::string>();
      if ( const auto state_error = verifyBaselineStateReceipt(
               state_receipt,
               qualification.at( "baseline_state_receipt_sha256" )
                   .get<std::string>() ) )
      {
        return *state_error;
      }
      validated_qualification.m_document = qualification;
      validated_qualification.m_sha256 =
          bytesSha256( qualification_bytes );
      return std::nullopt;
    }
    catch ( const std::exception& exception )
    {
      return std::string{ "baseline qualification invalid: " } +
             exception.what();
    }
  }

  [[nodiscard]] std::optional<std::string> writeM5ComparisonReceipt(
      const Arguments&                                          arguments,
      const M5BaselineQualification&                            validated,
      const phad::apps::internal::M5RpePrefixDiagnosticReceipt& intervention )
  {
    try
    {
      const nlohmann::json&       qualification = validated.m_document;
      const std::filesystem::path baseline_receipt_path =
          qualification.at( "baseline_state_receipt" ).get<std::string>();
      const std::string baseline_receipt_sha =
          qualification.at( "baseline_state_receipt_sha256" )
              .get<std::string>();
      if ( const auto state_error = verifyBaselineStateReceipt(
               baseline_receipt_path, baseline_receipt_sha ) )
      {
        return *state_error;
      }
      const std::string current_bytes =
          phad::apps::internal::m5RpePrefixDiagnosticReceiptJson(
              intervention );
      const nlohmann::ordered_json current_ordered =
          nlohmann::ordered_json::parse( current_bytes );
      if ( current_ordered.at( "schema" ) !=
               "m5_51_prefix_diagnostic_receipt_v1" ||
           current_ordered.at( "hook_frame_index" ) != 6U ||
           !current_ordered.at( "hook_executed" ).get<bool>() ||
           !current_ordered.at( "failure_reason" ).get<std::string>().empty() )
      {
        return "M5 #51 intervention state receipt contract mismatch";
      }
      for ( const std::string_view field :
            { "pre_hook_state_v1", "post_hook_state_v1", "post_state_v1" } )
      {
        if ( const auto state_error =
                 verifyCanonicalState( current_ordered.at( field ), field ) )
        {
          return state_error;
        }
      }

      const nlohmann::json baseline =
          nlohmann::json::parse( readFileBytes( baseline_receipt_path ) );
      const nlohmann::json current = nlohmann::json::parse( current_bytes );

      if ( baseline.at( "pre_hook_state_v1" ) !=
           current.at( "pre_hook_state_v1" ) )
      {
        return "M5 #51 arm pre-hook states differ";
      }
      const nlohmann::json expected_hook_diff = nlohmann::json::array(
          { { { "op", "replace" },
              { "path", "/keyframe_selector/total_keyframes" },
              { "value", 2U } } } );
      if ( current.at( "hook_diff" ) != expected_hook_diff )
      {
        return "M5 #51 intervention hook diff is not the selector scalar";
      }

      const nlohmann::json& baseline_post =
          baseline.at( "post_state_v1" ).at( "canonical_json" );
      const nlohmann::json& intervention_post =
          current.at( "post_state_v1" ).at( "canonical_json" );
      const auto& intervention_diag = intervention_post.at( "session" )
                                          .at( "result" )
                                          .at( "diag" );
      if ( intervention_diag.size() != 43U ||
           intervention_diag.at( 7 ).at( "is_keyframe" ).get<bool>() )
      {
        return "M5 #51 intervention did not make i=7 non-keyframe";
      }

      nlohmann::ordered_json comparison;
      comparison[ "schema" ] = "phad.m5_51.prefix_comparison.v1";
      comparison[ "baseline_qualification_sha256" ] =
          validated.m_sha256;
      comparison[ "source_id" ] = qualification.at( "source_id" );
      comparison[ "source_manifest_sha256" ] =
          qualification.at( "source_manifest_sha256" );
      comparison[ "source_archive_sha256" ] =
          qualification.at( "source_archive_sha256" );
      comparison[ "base_bundle_sha256" ] =
          qualification.at( "base_bundle_sha256" );
      comparison[ "source_verification_receipt_sha256" ] =
          qualification.at( "source_verification_receipt_sha256" );
      comparison[ "baseline_state_receipt_sha256" ] = baseline_receipt_sha;
      comparison[ "intervention_state_receipt_sha256" ] =
          fileSha256( arguments.m5_51_state_receipt );
      comparison[ "pre_hook_state_sha256" ] =
          current.at( "pre_hook_state_v1" ).at( "sha256" );
      comparison[ "intervention_hook_diff" ] = current.at( "hook_diff" );
      comparison[ "baseline_post_state_sha256" ] =
          baseline.at( "post_state_v1" ).at( "sha256" );
      comparison[ "intervention_post_state_sha256" ] =
          current.at( "post_state_v1" ).at( "sha256" );
      comparison[ "post_state_diff" ] =
          nlohmann::json::diff( baseline_post, intervention_post );
      comparison[ "intervention_i7_is_keyframe" ] = false;
      if ( !writeTextFile( arguments.m5_51_comparison_receipt,
                           comparison.dump() ) )
      {
        return "failed to write M5 #51 comparison receipt";
      }
      return std::nullopt;
    }
    catch ( const std::exception& exception )
    {
      return std::string{ "M5 #51 comparison failed: " } + exception.what();
    }
  }
#endif

  [[nodiscard]] phad::bench::ConfigSnapshot flattenConfig(
      const Arguments&                           arguments,
      const phad::apps::OfflineVoSessionOptions& session )
  {
    phad::bench::ConfigSnapshot snap;
    const auto&                 tracker   = session.tracker;
    const auto&                 estimator = session.estimator;

    snap.set( "tracker.max_tracks",
              static_cast<std::int64_t>( tracker.max_tracks ) );
    snap.set( "tracker.quality_level", tracker.quality_level );
    snap.set( "tracker.min_distance_px", tracker.min_distance_px );
    snap.set( "tracker.mask_radius_px",
              static_cast<std::int64_t>( tracker.mask_radius_px ) );
    snap.set( "tracker.lk_window_px",
              static_cast<std::int64_t>( tracker.lk_window_px ) );
    snap.set( "tracker.lk_pyramid_levels",
              static_cast<std::int64_t>( tracker.lk_pyramid_levels ) );
    snap.set( "tracker.forward_backward_px", tracker.forward_backward_px );
    snap.set( "tracker.max_epipolar_px", tracker.max_epipolar_px );
    snap.set( "tracker.min_disparity_px", tracker.min_disparity_px );
    snap.set( "tracker.min_depth_m", tracker.min_depth_m );
    snap.set( "tracker.max_depth_m", tracker.max_depth_m );
    snap.set( "tracker.stereo_sad_half_win_px",
              static_cast<std::int64_t>( tracker.stereo_sad_half_win_px ) );
    snap.set( "tracker.stereo_row_tol_px",
              static_cast<std::int64_t>( tracker.stereo_row_tol_px ) );
    snap.set( "tracker.stereo_bidir_px", tracker.stereo_bidir_px );
    snap.set( "tracker.stereo_uniq_ratio", tracker.stereo_uniq_ratio );
    snap.set( "tracker.stereo_check_bidir", tracker.stereo_check_bidir );
    // enable_clahe / enable_median_flow / enable_fransac / enable_census
    // are CLI-only attribution switches: not in config_hash (probe
    // convention), use --config-label to distinguish artifact directories.

    snap.set( "estimator.window_size",
              static_cast<std::int64_t>( estimator.window_size ) );
    snap.set( "estimator.min_landmark_observations",
              static_cast<std::int64_t>( estimator.min_landmark_observations ) );
    snap.set( "estimator.min_shared_landmarks",
              static_cast<std::int64_t>( estimator.min_shared_landmarks ) );
    snap.set( "estimator.stereo_sigma_px", estimator.stereo_sigma_px );
    snap.set( "estimator.huber_k_px", estimator.huber_k_px );
    snap.set( "estimator.prior_rotation_sigma_rad",
              estimator.prior_rotation_sigma_rad );
    snap.set( "estimator.prior_translation_sigma_m",
              estimator.prior_translation_sigma_m );
    snap.set( "estimator.min_seed_observations",
              static_cast<std::int64_t>( estimator.min_seed_observations ) );
    snap.set( "estimator.min_track_observations_for_seed",
              static_cast<std::int64_t>(
                  estimator.min_track_observations_for_seed ) );
    snap.set( "estimator.enable_pnp_init", estimator.enable_pnp_init );
    snap.set( "estimator.pnp_reproj_px", estimator.pnp_reproj_px );
    snap.set( "estimator.pnp_confidence", estimator.pnp_confidence );
    snap.set( "estimator.min_pnp_inliers",
              static_cast<std::int64_t>( estimator.min_pnp_inliers ) );
    snap.set( "estimator.enable_outlier_cull", estimator.enable_outlier_cull );
    snap.set( "estimator.outlier_avg_reproj_px",
              estimator.outlier_avg_reproj_px );
    snap.set( "estimator.enable_outlier_reopt",
              estimator.enable_outlier_reopt );
    snap.set( "estimator.max_outlier_reopts",
              static_cast<std::int64_t>( estimator.max_outlier_reopts ) );
    snap.set( "estimator.block_culled_rebirth",
              estimator.block_culled_rebirth );
    snap.set( "estimator.hanging_landmark_gate_m",
              estimator.hanging_landmark_gate_m );
    snap.set( "estimator.far_return_refresh_px",
              estimator.far_return_refresh_px );
    snap.set( "estimator.enable_moving_bootstrap",
              estimator.m_enable_moving_bootstrap );
    if ( estimator.enable_pnp_cheirality_retry )
    {
      snap.set( "estimator.enable_pnp_cheirality_retry", true );
    }
    snap.set( "estimator.velocity_prior_sigma_mps",
              estimator.m_velocity_prior_sigma_mps );

    snap.set( "session.dataset_format",
              std::string( phad::apps::internal::voBenchDatasetConfigToken(
                  arguments.dataset_format ) ) );
    snap.set( "session.drop_culled_tracks", session.drop_culled_tracks );
    snap.set( "session.skip_drop_min_culled",
              static_cast<std::int64_t>( session.skip_drop_min_culled ) );
    snap.set( "session.zombie_drop_age",
              static_cast<std::int64_t>( session.zombie_drop_age ) );
    if ( session.max_frames.has_value() )
    {
      snap.set( "session.max_frames",
                static_cast<std::int64_t>( *session.max_frames ) );
    }

    snap.set( "eval.max_dt_ms", arguments.max_dt_ms );
    snap.set( "eval.min_match_rate", arguments.min_match_rate );
    snap.set( "eval.rpe_delta_s", arguments.rpe_delta_s );
    return snap;
  }

  [[nodiscard]] phad::bench::StatBlock toStatBlock(
      const phad::eval::ErrorStats& stats )
  {
    return phad::bench::StatBlock{ .rmse   = stats.rmse,
                                   .mean   = stats.mean,
                                   .median = stats.median,
                                   .stddev = stats.stddev,
                                   .max    = stats.max };
  }

  [[nodiscard]] phad::bench::StageTiming toBenchTiming(
      const phad::apps::StageTiming& timing )
  {
    return phad::bench::StageTiming{ .mean = timing.mean,
                                     .p95  = timing.p95,
                                     .max  = timing.max };
  }

  [[nodiscard]] bool writeErrorsCsv( const std::filesystem::path& path,
                                     const phad::eval::AteReport& report )
  {
    std::ofstream stream( path, std::ios::trunc );
    if ( !stream )
    {
      return false;
    }
    stream << std::setprecision( std::numeric_limits<double>::max_digits10 );
    stream << "timestamp_ns,dt_ns,err_trans_m,err_rot_deg,est_x,est_y,est_z,"
              "gt_x,gt_y,gt_z\n";
    for ( const phad::eval::PoseErrorSample& sample : report.samples )
    {
      stream << sample.timestamp.nanoseconds() << ',' << sample.dt_ns << ','
             << sample.trans_m << ',' << sample.rot_deg << ','
             << sample.aligned_est_position.x() << ','
             << sample.aligned_est_position.y() << ','
             << sample.aligned_est_position.z() << ','
             << sample.gt_position.x() << ',' << sample.gt_position.y() << ','
             << sample.gt_position.z() << '\n';
    }
    stream.flush();
    return static_cast<bool>( stream );
  }

  [[nodiscard]] double coverageRate(
      const phad::apps::OfflineVoSessionResult& session )
  {
    if ( !session.trajectory.has_value() || session.trajectory->size() == 0U )
    {
      return 0.0;
    }
    const auto&  poses = session.trajectory->poses();
    const double pose_span_s =
        static_cast<double>( poses.back().timestamp.nanoseconds() -
                             poses.front().timestamp.nanoseconds() ) *
        1e-9;
    const double image_span_s =
        static_cast<double>( session.last_image_ts.nanoseconds() -
                             session.first_image_ts.nanoseconds() ) *
        1e-9;
    if ( image_span_s <= 0.0 )
    {
      return 0.0;
    }
    return pose_span_s / image_span_s;
  }

  [[nodiscard]] double imageSpanSeconds(
      const phad::apps::OfflineVoSessionResult& session )
  {
    if ( session.counts.image_frames == 0U )
    {
      return 0.0;
    }
    return static_cast<double>( session.last_image_ts.nanoseconds() -
                                session.first_image_ts.nanoseconds() ) *
           1e-9;
  }

  [[nodiscard]] int run( const Arguments& arguments )
  {
    const std::string sequence = resolveSequenceName( arguments );
    if ( sequence.empty() || sequence == "mav0" )
    {
      std::cerr << "sequence name is empty or 'mav0'; pass --sequence-name\n";
      return 2;
    }

    phad::apps::OfflineVoSessionOptions session_options;
    session_options.max_frames = arguments.max_frames;
    // Q1 Observe is a CLI-only side-channel and never enters flattenConfig.
    session_options.collect_gyro_observe = arguments.gyro_observe;
    if ( arguments.no_outlier_cull )
    {
      session_options.estimator.enable_outlier_cull = false;
    }
    if ( arguments.no_outlier_reopt )
    {
      session_options.estimator.enable_outlier_reopt = false;
    }
    // Default leaves estimator.block_culled_rebirth=true. Flag only for
    // A/B that re-enables same-id backproject; session still dropTracks
    // unless --no-drop-culled-tracks.
    if ( arguments.allow_culled_rebirth )
    {
      session_options.estimator.block_culled_rebirth = false;
    }
    if ( arguments.no_drop_culled_tracks )
    {
      session_options.drop_culled_tracks = false;
    }
    if ( arguments.skip_drop_min_culled.has_value() )
    {
      session_options.skip_drop_min_culled = *arguments.skip_drop_min_culled;
    }
    // Attribution A/B: Slice ⑥ frontend mechanism overrides.
    if ( arguments.tracker_quality_level.has_value() )
    {
      session_options.tracker.quality_level = *arguments.tracker_quality_level;
    }
    if ( arguments.tracker_lk_pyramid_levels.has_value() )
    {
      session_options.tracker.lk_pyramid_levels =
          *arguments.tracker_lk_pyramid_levels;
    }
    session_options.tracker.enable_clahe       = !arguments.no_clahe;
    session_options.tracker.enable_median_flow = !arguments.no_median_flow;
    session_options.tracker.enable_fransac     = !arguments.no_fransac;
    session_options.tracker.enable_census      = !arguments.no_census;
    session_options.tracker.enable_exposure_normalize =
        arguments.enable_exposure_norm;
    session_options.tracker.enable_zero_mean_sad =
        arguments.enable_zero_mean_sad;
    session_options.estimator.m_enable_moving_bootstrap =
        arguments.enable_moving_bootstrap;
    session_options.estimator.enable_pnp_cheirality_retry =
        arguments.enable_pnp_cheirality_retry;
    if ( arguments.min_seed_observations.has_value() )
    {
      session_options.estimator.min_seed_observations =
          *arguments.min_seed_observations;
    }
    if ( arguments.outlier_avg_reproj_px.has_value() )
    {
      session_options.estimator.outlier_avg_reproj_px =
          *arguments.outlier_avg_reproj_px;
    }
    if ( arguments.max_outlier_reopts.has_value() )
    {
      session_options.estimator.max_outlier_reopts =
          *arguments.max_outlier_reopts;
    }
    // Probe B path is CLI-only: not in flattenConfig / config_hash.
    session_options.probe_b_path = arguments.probe_b_path;
    // defer-drop-topk is CLI-only probe: not in flattenConfig / config_hash.
    if ( arguments.defer_drop_topk.has_value() )
    {
      session_options.defer_drop_topk = *arguments.defer_drop_topk;
    }
    // evict-skip-culled is CLI-only probe: not in flattenConfig / config_hash.
    session_options.evict_skip_culled = arguments.evict_skip_culled;
    // zombie-drop-age defaults to 5 and enters flattenConfig / config_hash.
    if ( arguments.zombie_drop_age.has_value() )
    {
      session_options.zombie_drop_age = *arguments.zombie_drop_age;
    }
    // Slice ⑦ hanging-landmark freshness gate; enters flattenConfig.
    if ( arguments.hanging_gate_m.has_value() )
    {
      session_options.estimator.hanging_landmark_gate_m =
          *arguments.hanging_gate_m;
    }
    // Slice ⑦ E12g far-return 3D refresh (proj-consistency gate); enters
    // flattenConfig. <= 0 disables (E13 pure-gate runs).
    if ( arguments.far_refresh_px.has_value() )
    {
      session_options.estimator.far_return_refresh_px =
          *arguments.far_refresh_px;
    }

#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
    const bool m5_51_diagnostic = !arguments.m5_51_state_receipt.empty();
    if ( m5_51_diagnostic )
    {
      session_options.collect_timing = false;
      if ( !session_options.max_frames.has_value() ||
           *session_options.max_frames != 43U )
      {
        std::cerr << "--m5-51-state-receipt requires --max-frames 43\n";
        return 2;
      }
      if ( !session_options.estimator.m_enable_moving_bootstrap )
      {
        std::cerr << "--m5-51-state-receipt requires "
                     "--estimator-enable-moving-bootstrap\n";
        return 2;
      }
      if ( session_options.estimator.enable_pnp_cheirality_retry )
      {
        std::cerr << "--m5-51-state-receipt requires PnP cheirality retry "
                     "to remain disabled\n";
        return 2;
      }
      if ( arguments.no_clahe || arguments.no_median_flow ||
           arguments.no_fransac || !arguments.no_census ||
           arguments.enable_exposure_norm ||
           arguments.enable_zero_mean_sad || arguments.gyro_observe ||
           !arguments.probe_b_path.empty() ||
           arguments.defer_drop_topk.has_value() ||
           arguments.evict_skip_culled )
      {
        std::cerr << "--m5-51-state-receipt requires all CLI-only behavior "
                     "switches to remain at formal defaults\n";
        return 2;
      }
      if ( session_options.collect_timing )
      {
        std::cerr << "--m5-51-state-receipt requires timing collection "
                     "to remain disabled\n";
        return 2;
      }
#if defined( PHAD_M5_51_REPLAY_DIAGNOSTIC_CLI_FIXTURE_TESTING )
      if ( !arguments.m5_51_selector_warmup )
      {
        std::cerr << "M5 #51 CLI fixture requires "
                     "--m5-51-selector-warmup\n";
        return 2;
      }
#endif
    }
#endif

    phad::apps::OfflineVoSessionOptions identity_options = session_options;
#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
    if ( m5_51_diagnostic )
    {
      // The 43-frame limit is the diagnostic execution bound, not an
      // algorithm/configuration change. It remains explicit in the state
      // receipt while the canonical formal configuration stays comparable
      // with the unbounded reference run.
      identity_options.max_frames.reset();
    }
#endif
    phad::bench::ConfigSnapshot config =
        flattenConfig( arguments, identity_options );
    const std::string config_hash = config.hash8();
    const std::string config_text = config.canonicalText();
    const std::string config_json = config.toJson();

#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
    M5BaselineQualification m5_51_baseline_qualification;
    if ( m5_51_diagnostic && config_hash != "0337287b" )
    {
      std::cerr << "M5 #51 diagnostic canonical config must be 0337287b, got "
                << config_hash << '\n';
      return 2;
    }
    if ( arguments.m5_51_selector_warmup )
    {
      const auto qualification_error = validateM5BaselineQualification(
          arguments, config_hash, config_text,
          m5_51_baseline_qualification );
      if ( qualification_error )
      {
        std::cerr << *qualification_error << '\n';
        return 2;
      }
    }
#endif

    std::vector<std::string>        warnings;
    const phad::bench::CodeIdentity code =
        phad::bench::queryGitIdentity( arguments.repo, warnings );
    if ( code.git_dirty )
    {
      warnings.emplace_back(
          "git working tree is dirty; not suitable as a formal baseline" );
    }

    const bool            use_override = !arguments.out_dir.empty();
    std::filesystem::path output_dir =
        use_override ? arguments.out_dir
                     : phad::bench::composeRunDir( arguments.bench_root,
                                                   sequence, code,
                                                   arguments.config_label,
                                                   config_hash );

    const phad::bench::OverwriteDecision overwrite =
        phad::bench::decideOverwrite( output_dir, code.git_dirty,
                                      arguments.force );
    if ( overwrite == phad::bench::OverwriteDecision::kRefuse )
    {
      std::cerr << "refusing to overwrite existing run at " << output_dir
                << " (pass --force)\n";
      return 3;
    }
    if ( overwrite == phad::bench::OverwriteDecision::kOverwriteWithWarning )
    {
      warnings.emplace_back(
          arguments.force
              ? "overwriting existing summary.json due to --force"
              : "overwriting existing summary.json on dirty tree" );
    }

    std::error_code ec;
    std::filesystem::create_directories( output_dir, ec );
    if ( ec )
    {
      std::cerr << "failed to create output dir: " << ec.message() << '\n';
      return 1;
    }

    phad::bench::RunMeta meta;
    meta.sequence              = sequence;
    meta.sequence_root         = arguments.sequence_root.string();
    meta.created_utc           = utcNowIso8601();
    meta.code                  = code;
    meta.config_json           = config_json;
    meta.config_label          = arguments.config_label;
    meta.config_hash           = config_hash;
    meta.config_canonical_text = config_text;
    meta.bench_root            = arguments.bench_root.string();
    meta.output_dir            = output_dir.string();
    meta.layout_template       = use_override ? "override" : "template";
    meta.warnings              = warnings;
    if ( !writeTextFile( output_dir / "meta.json", meta.toJson() ) )
    {
      std::cerr << "failed to write meta.json\n";
      return 1;
    }

    phad::apps::OfflineVoSessionResult session;
#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
    phad::apps::internal::M5RpePrefixDiagnosticReceipt m5_51_receipt;
    bool                                               m5_51_receipt_valid = true;
#endif
#if defined( PHAD_M5_51_REPLAY_DIAGNOSTIC_CLI_FIXTURE_TESTING )
    if ( m5_51_diagnostic )
    {
      const auto arm = arguments.m5_51_selector_warmup
                           ? phad::apps::internal::M5RpePrefixArm::
                                 kSelectorWarmup
                           : phad::apps::internal::M5RpePrefixArm::kBaseline;
      session        = phad::apps::testing::runM551ReplayDiagnosticCliFixture(
          m5_51_baseline_qualification.m_document
              .at( "baseline_state_receipt" )
              .get<std::string>(),
          arm, m5_51_receipt );
    }
    else
#endif
    {
      auto dataset = phad::apps::internal::openVoBenchDataset(
          arguments.dataset_format, arguments.sequence_root );
      if ( !dataset )
      {
        session.error =
            phad::apps::SessionError{ dataset.error().describe() };
      }
      else
      {
#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
        if ( m5_51_diagnostic )
        {
          const auto arm = arguments.m5_51_selector_warmup
                               ? phad::apps::internal::M5RpePrefixArm::
                                     kSelectorWarmup
                               : phad::apps::internal::M5RpePrefixArm::kBaseline;
          session        = phad::apps::internal::runM5RpePrefixDiagnostic(
              std::move( dataset ).value(), session_options, arm,
              m5_51_receipt );
        }
        else
#endif
        {
          session = phad::apps::runOfflineVoSession(
              std::move( dataset ).value(), session_options );
        }
      }
    }

#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
    if ( m5_51_diagnostic )
    {
      if ( !writeTextFile(
               arguments.m5_51_state_receipt,
               phad::apps::internal::m5RpePrefixDiagnosticReceiptJson(
                   m5_51_receipt ) ) )
      {
        std::cerr << "failed to write M5 #51 state receipt: "
                  << arguments.m5_51_state_receipt << '\n';
        return 1;
      }
      if ( !m5_51_receipt.m_hook_executed ||
           !m5_51_receipt.m_failure_reason.empty() )
      {
        m5_51_receipt_valid      = false;
        const std::string detail = !m5_51_receipt.m_failure_reason.empty()
                                       ? m5_51_receipt.m_failure_reason
                                       : "M5 #51 diagnostic hook did not execute";
        session.warnings.push_back( detail );
        std::cerr << detail << '\n';
      }
      if ( arguments.m5_51_selector_warmup && m5_51_receipt_valid )
      {
        if ( const auto comparison_error =
                 writeM5ComparisonReceipt( arguments,
                                           m5_51_baseline_qualification,
                                           m5_51_receipt ) )
        {
          m5_51_receipt_valid = false;
          session.warnings.push_back( *comparison_error );
          std::cerr << *comparison_error << '\n';
        }
      }
    }
#if defined( PHAD_M5_51_REPLAY_DIAGNOSTIC_CLI_FIXTURE_TESTING )
    if ( m5_51_diagnostic )
    {
      return m5_51_receipt_valid ? 0 : 1;
    }
#endif
#endif

    phad::bench::RunSummary summary;
    summary.sequence         = sequence;
    summary.git_commit_short = code.git_commit_short;
    summary.git_dirty        = code.git_dirty;
    summary.config_label     = arguments.config_label;
    summary.config_hash      = config_hash;
    summary.warnings         = warnings;
    summary.warnings.insert( summary.warnings.end(), session.warnings.begin(),
                             session.warnings.end() );
    summary.sync = phad::bench::SyncSummary{
        session.sync.pushed_left,
        session.sync.pushed_right,
        session.sync.emitted_stereo,
        session.sync.dropped_left,
        session.sync.dropped_right,
        session.sync.dropped_left_overflow,
        session.sync.dropped_right_overflow,
        session.sync.max_left_queue,
        session.sync.max_right_queue,
    };
    summary.trajectory.image_frames = session.counts.image_frames;
    summary.trajectory.ok           = session.counts.ok;
    summary.trajectory.rejected     = session.counts.rejected;
    summary.trajectory.failed       = session.counts.failed;
    summary.trajectory.poses_written =
        session.trajectory.has_value() ? session.trajectory->size() : 0U;
    summary.trajectory.completion_rate =
        session.counts.image_frames == 0U
            ? 0.0
            : static_cast<double>( session.counts.ok ) /
                  static_cast<double>( session.counts.image_frames );
    summary.trajectory.coverage_rate   = coverageRate( session );
    summary.trajectory.segments        = session.counts.segments;
    summary.trajectory.total_keyframes = session.counts.total_keyframes;
    summary.trajectory.total_track_only_frames =
        session.counts.total_track_only_frames;
    summary.robustness.rejected         = session.counts.rejected;
    summary.robustness.failed           = session.counts.failed;
    summary.robustness.low_connectivity = session.counts.low_connectivity;
    summary.robustness.reanchors        = session.counts.reanchors;
    summary.robustness.pnp_successes    = session.counts.pnp_successes;
    summary.robustness.pnp_fallbacks    = session.counts.pnp_fallbacks;
    summary.robustness.outliers_culled  = session.counts.outliers_culled;
    summary.robustness.outliers_culled_unique =
        session.counts.outliers_culled_unique;
    summary.robustness.outlier_reopts    = session.counts.outlier_reopts;
    summary.robustness.drops_skipped     = session.counts.drops_skipped;
    summary.robustness.deferred_drops    = session.counts.deferred_drops;
    summary.robustness.deferred_drop_ids = session.counts.deferred_drop_ids;
    summary.robustness.evictable_marked  = session.counts.evictable_marked;
    summary.robustness.tracks_evicted    = session.counts.tracks_evicted;
    summary.robustness.zombie_age_drops  = session.counts.zombie_age_drops;
    summary.robustness.zombie_age_drop_ids =
        session.counts.zombie_age_drop_ids;
    for ( const auto& row : session.diag )
    {
      summary.robustness.cheirality += row.num_cheirality;
    }
    summary.timing.wall_s     = session.wall_s;
    summary.timing.rectify    = toBenchTiming( session.timings.rectify );
    summary.timing.frontend   = toBenchTiming( session.timings.frontend );
    summary.timing.estimator  = toBenchTiming( session.timings.estimator );
    summary.timing.total      = toBenchTiming( session.timings.total );
    const double image_span_s = imageSpanSeconds( session );
    if ( session.wall_s > 0.0 && image_span_s > 0.0 )
    {
      summary.timing.rtf = image_span_s / session.wall_s;
    }

    const bool terminal_estimator_error =
        session.error.has_value() &&
        session.error->estimator_status.has_value() &&
        ( *session.error->estimator_status ==
              phad::estimator::UpdateStatus::kInvalidInput ||
          *session.error->estimator_status ==
              phad::estimator::UpdateStatus::kFailed );

    int exit_code = 0;
    if ( terminal_estimator_error )
    {
      summary.status = phad::bench::RunStatus::kFailed;
      summary.warnings.push_back( session.error->detail );
      std::cerr << session.error->detail << '\n';
      exit_code = 1;

      const auto write_partial_trajectory =
          [ &summary ]( const std::filesystem::path&                   path,
                        const std::optional<phad::common::Trajectory>& trajectory ) {
            if ( trajectory.has_value() )
            {
              if ( const auto error = phad::eval::writeTum( path, *trajectory ) )
              {
                summary.warnings.push_back( error->describe() );
                std::cerr << "write partial tum failed: "
                          << error->describe() << '\n';
                return false;
              }
              return true;
            }
            if ( !writeTextFile( path, "" ) )
            {
              const std::string detail =
                  "failed to write empty trajectory: " + path.string();
              summary.warnings.push_back( detail );
              std::cerr << detail << '\n';
              return false;
            }
            return true;
          };

      const bool est_written = write_partial_trajectory(
          output_dir / "est.tum", session.trajectory );
      bool diag_written = false;
      if ( est_written )
      {
        if ( const auto error = phad::apps::writeDiagCsv(
                 output_dir / "diag.csv", session.diag ) )
        {
          summary.warnings.push_back( error->detail );
          std::cerr << error->detail << '\n';
        }
        else
        {
          diag_written = true;
        }
      }
      if ( diag_written )
      {
        static_cast<void>( write_partial_trajectory(
            output_dir / "kf.tum", session.kf_trajectory ) );
      }
    }
    else if ( session.error.has_value() || !session.trajectory.has_value() )
    {
      summary.status = phad::bench::RunStatus::kFailed;
      if ( session.error.has_value() )
      {
        summary.warnings.push_back( session.error->detail );
        std::cerr << session.error->detail << '\n';
      }
      else
      {
        constexpr std::string_view kNoAcceptedPoses =
            "no accepted poses to write";
        summary.warnings.emplace_back( kNoAcceptedPoses );
        std::cerr << kNoAcceptedPoses << '\n';
      }
      exit_code = 1;
    }
    else
    {
      if ( const auto write_error = phad::eval::writeTum(
               output_dir / "est.tum", *session.trajectory ) )
      {
        summary.status = phad::bench::RunStatus::kFailed;
        summary.warnings.push_back( write_error->describe() );
        std::cerr << "write tum failed: " << write_error->describe() << '\n';
        exit_code = 1;
      }
      else if ( const auto diag_error = phad::apps::writeDiagCsv(
                    output_dir / "diag.csv", session.diag ) )
      {
        summary.status = phad::bench::RunStatus::kFailed;
        summary.warnings.push_back( diag_error->detail );
        std::cerr << diag_error->detail << '\n';
        exit_code = 1;
      }
      else
      {
        const auto& groundtruth_root =
            arguments.dataset_format == DatasetFormat::kEuroc
                ? arguments.gt_euroc
                : arguments.gt_tum_vi;
        auto gt = phad::apps::internal::openVoBenchGroundtruth(
            arguments.dataset_format, groundtruth_root );
        if ( !gt )
        {
          summary.status = phad::bench::RunStatus::kEvalFailed;
          summary.warnings.push_back( gt.error().describe() );
          std::cerr << gt.error().describe() << '\n';
          exit_code = 1;
        }
        else
        {
          phad::eval::AssociationOptions association_options;
          association_options.max_dt_ns = static_cast<std::int64_t>(
              std::llround( arguments.max_dt_ms * 1'000'000.0 ) );
          association_options.min_match_rate = arguments.min_match_rate;

          phad::eval::AteOptions ate_options;
          ate_options.association = association_options;
          auto ate                = phad::eval::computeAte( *session.trajectory, gt.value(),
                                                            ate_options );

          phad::eval::RpeOptions rpe_options;
          rpe_options.association = association_options;
          rpe_options.delta_ns    = static_cast<std::int64_t>(
              std::llround( arguments.rpe_delta_s * 1'000'000'000.0 ) );
          auto rpe = phad::eval::computeRpe( *session.trajectory, gt.value(),
                                             rpe_options );

          if ( !ate || !rpe )
          {
            summary.status = phad::bench::RunStatus::kEvalFailed;
            if ( !ate )
            {
              summary.warnings.push_back( ate.error().describe() );
              std::cerr << ate.error().describe() << '\n';
            }
            if ( !rpe )
            {
              summary.warnings.push_back( rpe.error().describe() );
              std::cerr << rpe.error().describe() << '\n';
            }
            exit_code = 1;
          }
          else
          {
            summary.ate = phad::bench::MetricReport{
                .trans   = toStatBlock( ate.value().trans_m ),
                .rot_deg = toStatBlock( ate.value().rot_deg ),
            };
            summary.rpe = phad::bench::MetricReport{
                .trans   = toStatBlock( rpe.value().trans_m ),
                .rot_deg = toStatBlock( rpe.value().rot_deg ),
            };
            if ( arguments.write_errors_csv &&
                 !writeErrorsCsv( output_dir / "errors.csv", ate.value() ) )
            {
              summary.warnings.emplace_back( "failed to write errors.csv" );
            }

            if ( session.counts.failed > 0U )
            {
              summary.status =
                  phad::bench::RunStatus::kCompletedWithFailures;
            }
            else if ( !summary.warnings.empty() )
            {
              summary.status =
                  phad::bench::RunStatus::kCompletedWithWarnings;
            }
            else
            {
              summary.status = phad::bench::RunStatus::kCompleted;
            }
            exit_code = 0;
          }
        }
      }
    }

    // kf.tum: independent of the est.tum/diag/eval chain above.
    if ( exit_code == 0 && session.kf_trajectory.has_value() )
    {
      if ( const auto kf_error = phad::eval::writeTum(
               output_dir / "kf.tum", *session.kf_trajectory ) )
      {
        summary.status = phad::bench::RunStatus::kFailed;
        summary.warnings.push_back( kf_error->describe() );
        std::cerr << "write kf tum failed: " << kf_error->describe() << '\n';
        exit_code = 1;
      }
    }

    if ( exit_code == 0 && arguments.gyro_observe )
    {
      if ( const auto error = phad::apps::writeGyroObserveCsvs(
               output_dir / "gyro_packets.csv",
               output_dir / "gyro_samples.csv", session.gyro_observe ) )
      {
        summary.status = phad::bench::RunStatus::kFailed;
        summary.warnings.push_back( error->detail );
        std::cerr << error->detail << '\n';
        exit_code = 1;
      }
    }

#if defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
    if ( m5_51_diagnostic && !m5_51_receipt_valid )
    {
      summary.status = phad::bench::RunStatus::kFailed;
      exit_code      = 1;
    }
#endif

    // failed / eval_failed still write summary.json for the regression table.
    if ( !writeTextFile( output_dir / "summary.json", summary.toJson() ) )
    {
      std::cerr << "failed to write summary.json\n";
      return 1;
    }

    std::cout << "status=" << phad::bench::runStatusToString( summary.status )
              << '\n'
              << "sequence=" << sequence << '\n'
              << "output_dir=" << output_dir.string() << '\n'
              << "config_hash=" << config_hash << '\n'
              << "completion_rate=" << summary.trajectory.completion_rate
              << '\n'
              << "coverage_rate=" << summary.trajectory.coverage_rate << '\n';
    if ( summary.ate.has_value() )
    {
      std::cout << "ate_trans_rmse=" << summary.ate->trans.rmse << '\n';
    }
    if ( summary.rpe.has_value() )
    {
      std::cout << "rpe_trans_rmse=" << summary.rpe->trans.rmse << '\n';
    }
    if ( summary.timing.rtf.has_value() )
    {
      std::cout << "rtf=" << *summary.timing.rtf << '\n';
    }
    std::cout << "wall_s=" << summary.timing.wall_s << '\n';
    return exit_code;
  }

}  // namespace

int main( int argc, char** argv )
{
  Arguments arguments;
  if ( !parseArguments( argc, argv, arguments ) )
  {
    std::cerr << kUsage;
    return 2;
  }

  try
  {
    return run( arguments );
  }
  catch ( const std::exception& exception )
  {
    std::cerr << "phad_vo_bench error: " << exception.what() << '\n';
    return 1;
  }
}
