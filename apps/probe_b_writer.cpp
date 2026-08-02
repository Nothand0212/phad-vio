#include "apps/probe_b_writer.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace phad::apps
{
  namespace
  {

    [[nodiscard]] std::string formatDouble( double value )
    {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision( 6 ) << value;
      return stream.str();
    }

    [[nodiscard]] std::string formatUint64Array(
        const std::vector<std::uint64_t>& values )
    {
      std::string out = "[";
      for ( std::size_t index = 0; index < values.size(); ++index )
      {
        if ( index > 0U )
        {
          out += ',';
        }
        out += std::to_string( values[ index ] );
      }
      out += ']';
      return out;
    }

    [[nodiscard]] std::string formatShiftTopArray(
        const std::vector<ProbeBShiftTop>& entries )
    {
      std::string out = "[";
      for ( std::size_t index = 0; index < entries.size(); ++index )
      {
        if ( index > 0U )
        {
          out += ',';
        }
        out += "{\"key\":" + std::to_string( entries[ index ].key ) + ",\"dt_m\":"
               + formatDouble( entries[ index ].dt_m ) + '}';
      }
      out += ']';
      return out;
    }

    void appendCommaIfNeeded( std::string& json, bool& first )
    {
      if ( !first )
      {
        json += ',';
      }
      first = false;
    }

    [[nodiscard]] std::string serializeProbeBFrame( const ProbeBFrame& frame )
    {
      std::string json = "{";
      bool        first = true;

      json += "\"i\":" + std::to_string( frame.i );
      first = false;

      appendCommaIfNeeded( json, first );
      json += "\"ts_ns\":" + std::to_string( frame.ts_ns );

      if ( frame.culled_ids.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"culled_ids\":" + formatUint64Array( frame.culled_ids.value() );
      }
      if ( frame.zombie_track_n.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"zombie_track_n\":"
                + std::to_string( frame.zombie_track_n.value() );
      }
      if ( frame.rejected_block_n.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"rejected_block_n\":"
                + std::to_string( frame.rejected_block_n.value() );
      }
      if ( frame.new_lm.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"new_lm\":" + std::to_string( frame.new_lm.value() );
      }
      if ( frame.shared.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"shared\":" + std::to_string( frame.shared.value() );
      }
      if ( frame.num_obs.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"num_obs\":" + std::to_string( frame.num_obs.value() );
      }
      if ( frame.lm_iterations.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"lm_iterations\":"
                + std::to_string( frame.lm_iterations.value() );
      }
      if ( frame.shift_m.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"shift_m\":" + formatDouble( frame.shift_m.value() );
      }
      if ( frame.shift_top.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"shift_top\":" + formatShiftTopArray( frame.shift_top.value() );
      }
      if ( frame.res_mean_px.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"res_mean_px\":" + formatDouble( frame.res_mean_px.value() );
      }
      if ( frame.res_max_px.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"res_max_px\":" + formatDouble( frame.res_max_px.value() );
      }
      if ( frame.res_max_id.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"res_max_id\":" + std::to_string( frame.res_max_id.value() );
      }
      if ( frame.drops_skipped_this_frame.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"drops_skipped_this_frame\":"
                + std::string( frame.drops_skipped_this_frame.value() ? "true"
                                                                        : "false" );
      }

      json += "}\n";
      return json;
    }

  }  // namespace

  ProbeBWriter::ProbeBWriter( const std::filesystem::path& path )
      : m_out( path, std::ios::out | std::ios::trunc )
  {
    if ( !m_out )
    {
      throw std::runtime_error(
          "failed to open probe_b jsonl: " + path.string() );
    }
  }

  ProbeBWriter::~ProbeBWriter()
  {
    if ( m_out.is_open() )
    {
      m_out.flush();
      m_out.close();
    }
  }

  void ProbeBWriter::write( const ProbeBFrame& frame )
  {
    m_out << serializeProbeBFrame( frame );
    if ( !m_out )
    {
      throw std::runtime_error( "failed to write probe_b jsonl line" );
    }
  }

}  // namespace phad::apps
