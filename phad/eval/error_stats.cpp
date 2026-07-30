#include "phad/eval/error_stats.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

/**
 * @file error_stats.cpp
 * @brief 误差统计实现。
 */

namespace phad::eval
{

  ErrorStats computeStats( std::vector<double> errors )
  {
    ErrorStats stats;
    if ( errors.empty() )
    {
      return stats;
    }

    double sum         = 0.0;
    double squared_sum = 0.0;
    for ( const double error : errors )
    {
      sum += error;
      squared_sum += error * error;
    }
    const double count = static_cast<double>( errors.size() );
    stats.mean         = sum / count;
    stats.rmse         = std::sqrt( squared_sum / count );
    stats.stddev       = std::sqrt(
        std::max( 0.0, squared_sum / count - stats.mean * stats.mean ) );
    stats.max = *std::max_element( errors.begin(), errors.end() );

    const std::size_t middle = errors.size() / 2U;
    std::nth_element( errors.begin(),
                      errors.begin() + static_cast<std::ptrdiff_t>( middle ),
                      errors.end() );
    if ( errors.size() % 2U == 1U )
    {
      stats.median = errors[ middle ];
    }
    else
    {
      const double upper = errors[ middle ];
      const double lower = *std::max_element(
          errors.begin(), errors.begin() + static_cast<std::ptrdiff_t>( middle ) );
      stats.median = 0.5 * ( lower + upper );
    }
    return stats;
  }

}  // namespace phad::eval
