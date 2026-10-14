#pragma once

#include <bit>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include "apps/offline_vo_session.hpp"

namespace phad::apps
{
  namespace recovery_log
  {
    inline nlohmann::json observationValue( double value )
    {
      if ( std::isfinite( value ) )
      {
        return value;
      }
      std::ostringstream bits;
      bits << std::hex << std::setfill( '0' ) << std::setw( 16 )
           << std::bit_cast<std::uint64_t>( value );
      return { { "f64_bits", bits.str() } };
    }

    inline std::string_view name( estimator::InitializationPath value )
    {
      switch ( value )
      {
        case estimator::InitializationPath::kNone:
          return "None";
        case estimator::InitializationPath::kStatic:
          return "Static";
        case estimator::InitializationPath::kMoving:
          return "Moving";
      }
      throw std::runtime_error( "unknown InitializationPath" );
    }

    inline std::string_view name( estimator::InitializationOutcome value )
    {
      switch ( value )
      {
        case estimator::InitializationOutcome::kNotActive:
          return "NotActive";
        case estimator::InitializationOutcome::kPending:
          return "Pending";
        case estimator::InitializationOutcome::kRecoverableRejected:
          return "RecoverableRejected";
        case estimator::InitializationOutcome::kCommitted:
          return "Committed";
        case estimator::InitializationOutcome::kFatalError:
          return "FatalError";
      }
      throw std::runtime_error( "unknown InitializationOutcome" );
    }

    inline std::string_view name( estimator::InitializationTransactionResult value )
    {
      switch ( value )
      {
        case estimator::InitializationTransactionResult::kNotEvaluated:
          return "NotEvaluated";
        case estimator::InitializationTransactionResult::kEvidenceCommitted:
          return "EvidenceCommitted";
        case estimator::InitializationTransactionResult::kRootCommitted:
          return "RootCommitted";
        case estimator::InitializationTransactionResult::kRolledBack:
          return "RolledBack";
      }
      throw std::runtime_error( "unknown InitializationTransactionResult" );
    }

    inline std::string_view name( estimator::RecoveryBridgeOrigin value )
    {
      switch ( value )
      {
        case estimator::RecoveryBridgeOrigin::kNone:
          return "None";
        case estimator::RecoveryBridgeOrigin::kProcessStart:
          return "ProcessStart";
        case estimator::RecoveryBridgeOrigin::kVisualOutageRecovery:
          return "VisualOutageRecovery";
        case estimator::RecoveryBridgeOrigin::kMeasurementDiscontinuity:
          return "MeasurementDiscontinuity";
      }
      throw std::runtime_error( "unknown RecoveryBridgeOrigin" );
    }

    inline std::string_view name( estimator::RecoveryBridgeResult value )
    {
      switch ( value )
      {
        case estimator::RecoveryBridgeResult::kNotApplicable:
          return "NotApplicable";
        case estimator::RecoveryBridgeResult::kPending:
          return "Pending";
        case estimator::RecoveryBridgeResult::kEligible:
          return "Eligible";
        case estimator::RecoveryBridgeResult::kApplied:
          return "Applied";
        case estimator::RecoveryBridgeResult::kCleared:
          return "Cleared";
        case estimator::RecoveryBridgeResult::kFatalError:
          return "FatalError";
      }
      throw std::runtime_error( "unknown RecoveryBridgeResult" );
    }

    inline std::string_view name( estimator::RecoveryBridgeReason value )
    {
      switch ( value )
      {
        case estimator::RecoveryBridgeReason::kNone:
          return "None";
        case estimator::RecoveryBridgeReason::kAwaitingValidatedRoot:
          return "AwaitingValidatedRoot";
        case estimator::RecoveryBridgeReason::kStaticRootSelected:
          return "StaticRootSelected";
        case estimator::RecoveryBridgeReason::kMeasurementDiscontinuity:
          return "MeasurementDiscontinuity";
        case estimator::RecoveryBridgeReason::kTimestampMismatch:
          return "TimestampMismatch";
        case estimator::RecoveryBridgeReason::kInvalidAnchor:
          return "InvalidAnchor";
        case estimator::RecoveryBridgeReason::kYawUndefined:
          return "YawUndefined";
        case estimator::RecoveryBridgeReason::kInvalidTransform:
          return "InvalidTransform";
        case estimator::RecoveryBridgeReason::kContractError:
          return "ContractError";
        case estimator::RecoveryBridgeReason::kAnchorExpired:
          return "AnchorExpired";
      }
      throw std::runtime_error( "unknown RecoveryBridgeReason" );
    }

    template <typename T>
    inline nlohmann::json optionalValue( const std::optional<T>& value )
    {
      return value ? nlohmann::json( *value ) : nlohmann::json( nullptr );
    }

    inline nlohmann::json timestamp( const std::optional<common::Timestamp>& value )
    {
      return value ? nlohmann::json( value->nanoseconds() ) : nlohmann::json( nullptr );
    }
  }  // namespace recovery_log

  /// Pure projection of the captured input and same-call typed result.
  inline nlohmann::json recoveryBridgeLogRowJson( const VoDiagRow& row )
  {
    using namespace recovery_log;
    if ( !row.m_recovery_input )
    {
      throw std::runtime_error( "recovery bridge input was not collected" );
    }
    const auto& input = *row.m_recovery_input;
    if ( input.m_timestamp_ns != row.timestamp_ns ||
         input.m_is_keyframe != row.is_keyframe )
    {
      throw std::runtime_error( "recovery bridge input/result identity mismatch" );
    }
    auto observations = nlohmann::json::array();
    for ( const auto& observation : input.m_observations )
    {
      observations.push_back( { observation.id,
                                observationValue( observation.left_pixel.x() ),
                                observationValue( observation.left_pixel.y() ),
                                observationValue( observation.disparity_px ) } );
    }
    const auto&    init    = row.initialization;
    const auto&    bridge  = row.m_recovery_bridge;
    nlohmann::json lineage = nullptr;
    if ( bridge.m_lineage )
    {
      const auto& value = *bridge.m_lineage;
      lineage           = {
          { "source_segment_id", value.m_source_segment_id },
          { "source_frame_index", value.m_source_frame_index },
          { "source_ts_ns", value.m_source_timestamp.nanoseconds() },
          { "target_segment_id", value.m_target_segment_id },
          { "world_origin_segment_id", value.m_world_origin_segment_id },
      };
    }
    return {
        { "schema_version", 1 },
        { "i", input.m_processed_index },
        { "ts_ns", input.m_timestamp_ns },
        { "status", row.status },
        { "segment_id", row.segment_id },
        { "frontend", { { "is_keyframe", input.m_is_keyframe }, { "observations", std::move( observations ) } } },
        { "initialization",
          { { "path", name( init.m_path ) },
            { "phase", static_cast<unsigned>( init.m_phase ) },
            { "reason", static_cast<unsigned>( init.m_reason ) },
            { "outcome", name( init.m_outcome ) },
            { "attempt_id", optionalValue( init.m_attempt_id ) },
            { "current_graph", static_cast<unsigned>( init.m_current_graph ) },
            { "transaction", name( init.m_transaction ) } } },
        { "bridge",
          { { "origin", name( bridge.m_origin ) },
            { "result", name( bridge.m_result ) },
            { "reason", name( bridge.m_reason ) },
            { "lineage", std::move( lineage ) },
            { "anchor_ts_ns", timestamp( bridge.m_anchor_timestamp ) },
            { "root_ts_ns", timestamp( bridge.m_root_timestamp ) },
            { "attempt_id", optionalValue( bridge.m_attempt_id ) },
            { "transaction", name( bridge.m_transaction ) } } },
    };
  }

  inline std::optional<SessionError> writeRecoveryBridgeLog(
      const std::filesystem::path& path, const std::vector<VoDiagRow>& rows )
  {
    try
    {
      std::ofstream output;
      output.exceptions( std::ios::failbit | std::ios::badbit );
      output.open( path );
      for ( const auto& row : rows )
      {
        output << recoveryBridgeLogRowJson( row ).dump() << '\n';
      }
      output.close();
      return std::nullopt;
    }
    catch ( const std::exception& error )
    {
      return SessionError{ "recovery bridge log: " + std::string( error.what() ) };
    }
  }
}  // namespace phad::apps
