#pragma once

#include <filesystem>
#include <memory>

#include "phad/estimator/types.hpp"

namespace phad::apps
{

  class VioInitProbe
  {
  public:
    explicit VioInitProbe( const std::filesystem::path& path );
    ~VioInitProbe();

    VioInitProbe( const VioInitProbe& )            = delete;
    VioInitProbe& operator=( const VioInitProbe& ) = delete;
    VioInitProbe( VioInitProbe&& )                 = delete;
    VioInitProbe& operator=( VioInitProbe&& )      = delete;

    void write( const estimator::VioUpdateResult& update );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::apps
