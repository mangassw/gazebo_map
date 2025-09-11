/**
 * @file rpp_math.h
 * @brief 数学工具
 */
#ifndef RPP_PKG_RPP_MATH_H_
#define RPP_PKG_RPP_MATH_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include "rpp_pkg/rpp_vec2d.h"

namespace rpp_pkg {
namespace math {

constexpr double kMathEpsilon = 1e-10;

template <typename T>
T clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2); // 确保bound1 <= bound2
  }
  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

std::vector<rpp_pkg::vec2d::Vec2d> circleSegmentIntersection(const rpp_pkg::vec2d::Vec2d& p1,
                                                                    const rpp_pkg::vec2d::Vec2d& p2, double r);


double arcCenter(const rpp_pkg::vec2d::Vec2d& pt_prev, const rpp_pkg::vec2d::Vec2d& pt,
                 const rpp_pkg::vec2d::Vec2d& pt_next, bool is_cusp,
                 rpp_pkg::vec2d::Vec2d* center = nullptr);  
double crossProd(const rpp_pkg::vec2d::Vec2d& start_point, const rpp_pkg::vec2d::Vec2d& end_point_1,
                 const rpp_pkg::vec2d::Vec2d& end_point_2);
                 
                 

} // namespace math
} // namespace rpp_pkg

#endif // RPP_PKG_RPP_MATH_H_