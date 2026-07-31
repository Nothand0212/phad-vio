#include "phad/bench/config_snapshot.hpp"

#include <cstdio>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string_view>
#include <type_traits>

/**
 * @file config_snapshot.cpp
 * @brief ConfigSnapshot 规范化文本、hash 与 JSON 序列化。
 */

namespace phad::bench
{
  namespace
  {

    using json = nlohmann::json;

    void requireNonEmptyKey( std::string_view key )
    {
      if ( key.empty() )
      {
        throw std::invalid_argument( "ConfigSnapshot key must be non-empty" );
      }
    }

    [[nodiscard]] std::string formatDouble( double value )
    {
      char      buffer[ 64 ];
      const int written =
          std::snprintf( buffer, sizeof( buffer ), "%.17g", value );
      if ( written <= 0 || static_cast<std::size_t>( written ) >= sizeof( buffer ) )
      {
        throw std::runtime_error( "ConfigSnapshot failed to format double" );
      }
      return std::string( buffer );
    }

    [[nodiscard]] std::string formatValue( const ConfigSnapshot::Value& value )
    {
      return std::visit(
          []( const auto& item ) -> std::string {
            using T = std::decay_t<decltype( item )>;
            if constexpr ( std::is_same_v<T, std::string> )
            {
              return item;
            }
            else if constexpr ( std::is_same_v<T, double> )
            {
              return formatDouble( item );
            }
            else if constexpr ( std::is_same_v<T, std::int64_t> )
            {
              return std::to_string( item );
            }
            else if constexpr ( std::is_same_v<T, bool> )
            {
              return item ? "true" : "false";
            }
          },
          value );
    }

    [[nodiscard]] std::uint64_t fnv1a64( std::string_view text )
    {
      constexpr std::uint64_t kOffset = 14695981039346656037ULL;
      constexpr std::uint64_t kPrime  = 1099511628211ULL;
      std::uint64_t           hash    = kOffset;
      for ( const char ch : text )
      {
        hash ^= static_cast<std::uint64_t>( static_cast<unsigned char>( ch ) );
        hash *= kPrime;
      }
      return hash;
    }

    [[nodiscard]] std::string hash8FromText( std::string_view text )
    {
      const std::uint64_t hash = fnv1a64( text );
      char                buffer[ 17 ];
      const int           written = std::snprintf(
          buffer, sizeof( buffer ), "%016llx",
          static_cast<unsigned long long>( hash ) );
      if ( written != 16 )
      {
        throw std::runtime_error( "ConfigSnapshot failed to format hash" );
      }
      return std::string( buffer, 8 );
    }

  }  // namespace

  void ConfigSnapshot::set( std::string key, std::string value )
  {
    requireNonEmptyKey( key );
    m_values.insert_or_assign( std::move( key ), std::move( value ) );
  }

  void ConfigSnapshot::set( std::string key, double value )
  {
    requireNonEmptyKey( key );
    m_values.insert_or_assign( std::move( key ), value );
  }

  void ConfigSnapshot::set( std::string key, std::int64_t value )
  {
    requireNonEmptyKey( key );
    m_values.insert_or_assign( std::move( key ), value );
  }

  void ConfigSnapshot::set( std::string key, bool value )
  {
    requireNonEmptyKey( key );
    m_values.insert_or_assign( std::move( key ), value );
  }

  std::string ConfigSnapshot::canonicalText() const
  {
    std::string text;
    for ( const auto& [ key, value ] : m_values )
    {
      text += key;
      text += '=';
      text += formatValue( value );
      text += '\n';
    }
    return text;
  }

  std::string ConfigSnapshot::hash8() const
  {
    return hash8FromText( canonicalText() );
  }

  std::string ConfigSnapshot::toJson() const
  {
    json object = json::object();
    for ( const auto& [ key, value ] : m_values )
    {
      std::visit(
          [ &object, &key ]( const auto& item ) {
            using T = std::decay_t<decltype( item )>;
            if constexpr ( std::is_same_v<T, std::string> )
            {
              object[ key ] = item;
            }
            else if constexpr ( std::is_same_v<T, double> )
            {
              object[ key ] = item;
            }
            else if constexpr ( std::is_same_v<T, std::int64_t> )
            {
              object[ key ] = item;
            }
            else if constexpr ( std::is_same_v<T, bool> )
            {
              object[ key ] = item;
            }
          },
          value );
    }
    return object.dump();
  }

}  // namespace phad::bench
