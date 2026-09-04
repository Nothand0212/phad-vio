#pragma once

#include <atomic>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#if defined( _WIN32 )
#include <process.h>
#else
#include <unistd.h>
#endif

namespace phad::testing
{

  namespace detail
  {

    inline std::atomic<std::uint64_t> g_temp_directory_counter{ 0U };

    [[nodiscard]] inline std::uint64_t processId() noexcept
    {
#if defined( _WIN32 )
      return static_cast<std::uint64_t>( _getpid() );
#else
      return static_cast<std::uint64_t>( getpid() );
#endif
    }

    [[nodiscard]] inline std::string sanitizeTempDirectoryLabel(
        const std::string_view label )
    {
      std::string sanitized;
      sanitized.reserve( label.size() );
      for ( const char value : label )
      {
        const auto character = static_cast<unsigned char>( value );
        sanitized.push_back(
            std::isalnum( character ) != 0 || character == '-' ||
                    character == '_'
                ? static_cast<char>( character )
                : '-' );
      }
      return sanitized.empty() ? "test" : sanitized;
    }

  }  // namespace detail

  class ScopedTempDirectory final
  {
  public:
    explicit ScopedTempDirectory( const std::string_view label )
    {
      constexpr std::uint64_t kMaxAttempts = 1'024U;
      const std::string       prefix =
          detail::sanitizeTempDirectoryLabel( label ) + '-' +
          std::to_string( detail::processId() ) + '-';
      const std::filesystem::path temp_root =
          std::filesystem::temp_directory_path();

      for ( std::uint64_t attempt = 0U; attempt < kMaxAttempts; ++attempt )
      {
        const std::uint64_t token =
            detail::g_temp_directory_counter.fetch_add(
                1U, std::memory_order_relaxed );
        const std::filesystem::path candidate =
            temp_root / ( prefix + std::to_string( token ) );
        std::error_code error;
        if ( std::filesystem::create_directory( candidate, error ) )
        {
          m_path = candidate;
          return;
        }
        if ( error && error != std::errc::file_exists )
        {
          throw std::filesystem::filesystem_error(
              "cannot create scoped temporary directory", candidate, error );
        }
      }
      throw std::runtime_error(
          "cannot allocate a unique scoped temporary directory" );
    }

    ~ScopedTempDirectory() noexcept
    {
      std::error_code ignored;
      std::filesystem::remove_all( m_path, ignored );
    }

    ScopedTempDirectory( const ScopedTempDirectory& )            = delete;
    ScopedTempDirectory& operator=( const ScopedTempDirectory& ) = delete;
    ScopedTempDirectory( ScopedTempDirectory&& )                 = delete;
    ScopedTempDirectory& operator=( ScopedTempDirectory&& )      = delete;

    [[nodiscard]] const std::filesystem::path& path() const noexcept
    {
      return m_path;
    }

  private:
    std::filesystem::path m_path;
  };

}  // namespace phad::testing
