#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <utility>
#include <numbers>
#include <stdexcept>

namespace vec2{
  using Real = double;
  constexpr Real eps = 1e-9;

  // using Real = float;
  // constexpr Real eps = 1e-5f;
  
  struct vec2 {
  public:
    Real x = 0.0, y = 0.0;

    vec2() {}

    vec2(Real X, Real Y) : x(X), y(Y) {}

    template<typename T>
    vec2(std::pair<T, T> XY) : x(static_cast<Real>(XY.first)), y(static_cast<Real>(XY.second)) {}

    template<typename T>
    vec2(std::array<T, 2> XY) : x(static_cast<Real>(XY[0])), y(static_cast<Real>(XY[1])) {}

    vec2 operator + (const vec2& other) const {
      return vec2(x + other.x, y + other.y);
    }

    vec2 operator - (const vec2& other) const {
      return vec2(x - other.x, y - other.y);
    }

    vec2 operator * (const Real& other) const {
      return vec2(x * other, y * other);
    }

    vec2 operator / (const Real& other) const {
      if (std::abs(other) < eps) {
        throw std::runtime_error("Vec2Error: Division by Zero");
      } return vec2(x / other, y / other);
    }

    Real magnitude() const {
      return std::sqrt(x * x + y * y);
    }

    vec2 normalize() const {
      Real mag = magnitude();
      if (mag > eps) {
        return vec2(x / mag, y / mag);
      } else return vec2(0.0, 0.0);
    }

    Real dot (const vec2& other) const {
      return x * other.x + y * other.y;
    }

    Real cross (const vec2& other) const {
      return x * other.y - y * other.x;
    }

    vec2 project (const vec2& other) const {
      // projection of current vector in direction of other
      Real magOth = other.magnitude();
      if (magOth < eps) return vec2(0.0, 0.0);
      Real projection = dot(other) / magOth;
      return other.normalize() * projection;
    }

    Real angleTo (const vec2& other) const {
      // angle to other vector (measured anti clockwise)
      Real angle = std::atan2(cross(other), dot(other));
      if (angle < 0) angle += 2.0 * M_PI;
      return angle;
    }

    std::array<float, 2> floatify() const {
      // floatify for SDL2
      std::array<float, 2> floated{static_cast<float>(x), static_cast<float>(y)};
      return floated;
    }

    friend std::ostream& operator << (std::ostream& os, const vec2& u) {
      os << "(" << u.x << ", " << u.y << ")";
      return os;
    }
  };

  inline vec2 operator * (const Real k, const vec2 u) {
    return vec2(u.x * k, u.y * k);
  };
};
