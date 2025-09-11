/**
 * @file rpp_vec2d.cpp
 * @brief 向量工具
 */
#include <cmath>
#include "rpp_pkg/rpp_vec2d.h"
#include "rpp_pkg/rpp_math.h"


namespace rpp_pkg {
namespace vec2d {

Vec2d Vec2d::createUnitVec2d(const double angle)
{
  return Vec2d(std::cos(angle), std::sin(angle));
}

double Vec2d::length() const
{
  return std::hypot(x_, y_);
}

double Vec2d::lengthSquare() const
{
  return x_ * x_ + y_ * y_;
}

double Vec2d::angle() const
{
  return std::atan2(y_, x_);
}

void Vec2d::normalize()
{
  const double l = length();
  if (l > rpp_pkg::math::kMathEpsilon)
  {
    x_ /= l;
    y_ /= l;
  }
}

double Vec2d::distanceTo(const Vec2d& other) const
{
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Vec2d::distanceSquareTo(const Vec2d& other) const
{
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

double Vec2d::crossProd(const Vec2d& other) const
{
  return x_ * other.y() - y_ * other.x();
}

double Vec2d::innerProd(const Vec2d& other) const
{
  return x_ * other.x() + y_ * other.y();
}

Vec2d Vec2d::rotate(const double angle) const
{
  return Vec2d(x_ * cos(angle) - y_ * sin(angle), x_ * sin(angle) + y_ * cos(angle));
}

void Vec2d::selfRotate(const double angle)
{
  double tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2d Vec2d::operator+(const Vec2d& other) const
{
  return Vec2d(x_ + other.x(), y_ + other.y());
}

Vec2d Vec2d::operator-() const
{
  return Vec2d(-x_, -y_);
}

Vec2d Vec2d::operator-(const Vec2d& other) const
{
  return Vec2d(x_ - other.x(), y_ - other.y());
}

Vec2d Vec2d::operator*(const double ratio) const
{
  return Vec2d(x_ * ratio, y_ * ratio);
}

Vec2d Vec2d::operator/(const double ratio) const
{
  //CHECK_GT(std::abs(ratio), rpp_pkg::math::kMathEpsilon);
  if (std::abs(ratio) <= rpp_pkg::math::kMathEpsilon) {
    // 处理错误：记录日志并返回错误状态
  }
  return Vec2d(x_ / ratio, y_ / ratio);
}

Vec2d& Vec2d::operator+=(const Vec2d& other)
{
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2d& Vec2d::operator-=(const Vec2d& other)
{
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2d& Vec2d::operator*=(const double ratio)
{
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2d& Vec2d::operator/=(const double ratio)
{
  //CHECK_GT(std::abs(ratio), rpp_pkg::math::kMathEpsilon);
  if (std::abs(ratio) <= rpp_pkg::math::kMathEpsilon) {
    // 处理错误：记录日志并返回错误状态
  }
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2d::operator==(const Vec2d& other) const
{
  return (std::abs(x_ - other.x()) < rpp_pkg::math::kMathEpsilon &&
          std::abs(y_ - other.y()) < rpp_pkg::math::kMathEpsilon);
}

Vec2d operator*(const double ratio, const Vec2d& vec)
{
  return vec * ratio;
}



    
} // namespace vec2d
} // namespace rpp_pkg