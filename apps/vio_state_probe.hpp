#pragma once

#include <filesystem>
#include <memory>

#include "phad/estimator/types.hpp"

namespace phad::apps
{

  class VioStateProbe
  {
  public:
    explicit VioStateProbe( const std::filesystem::path& path );
    ~VioStateProbe();

    VioStateProbe( const VioStateProbe& )            = delete;
    VioStateProbe& operator=( const VioStateProbe& ) = delete;
    VioStateProbe( VioStateProbe&& )                 = delete;
    VioStateProbe& operator=( VioStateProbe&& )      = delete;

    void write( bool                              is_keyframe,
                const estimator::VioUpdateResult& update );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::apps
