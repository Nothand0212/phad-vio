#pragma once

#include <filesystem>
#include <memory>

#include "phad/estimator/types.hpp"

namespace phad::apps
{

  class FixedLagShadowProbe
  {
  public:
    explicit FixedLagShadowProbe( const std::filesystem::path& path );
    ~FixedLagShadowProbe();

    FixedLagShadowProbe( const FixedLagShadowProbe& )            = delete;
    FixedLagShadowProbe& operator=( const FixedLagShadowProbe& ) = delete;
    FixedLagShadowProbe( FixedLagShadowProbe&& )                 = delete;
    FixedLagShadowProbe& operator=( FixedLagShadowProbe&& )      = delete;

    void write( const estimator::VioUpdateResult& update );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::apps
