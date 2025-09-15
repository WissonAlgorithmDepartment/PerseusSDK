/**
 * @file math_common.hpp
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.1
 * @date: 2025-05-07
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Enhanced mathematics utilities for robotics applications
 */
#pragma once

#include <cmath>
#include <limits>
#include <type_traits>
#include <numbers>

namespace wisson_math {

// ----------------------------
// Type Aliases
// ----------------------------
using float32_t = float;
using float64_t = double;


// ----------------------------
// Mathematical Constants (C++20标准优化版)
// ----------------------------
template<typename T>
constexpr T Pi() noexcept {
    static_assert(std::is_floating_point_v<T>, 
                 "Pi<T> requires floating point type");
    return std::numbers::pi_v<T>;
}

template<typename T>
constexpr T TwoPi() noexcept { return std::numbers::pi_v<T> * static_cast<T>(2); }

template<typename T>
constexpr T HalfPi() noexcept { return std::numbers::pi_v<T> / static_cast<T>(2); }


// ----------------------------
// Unit Conversions
// ----------------------------
constexpr double DEG_1_IN_RAD   = 0.0174532925199;
constexpr double DEG_0_1_IN_RAD = 0.00174532925199;

template<typename T>
constexpr T deg_to_rad(T degrees) noexcept 
{
    static_assert(std::is_floating_point_v<T>,
                 "DegToRad requires floating point type");
    return degrees * std::numbers::pi_v<T> / static_cast<T>(180);
}

template<typename T>
constexpr T rad_to_deg(T radians) noexcept 
{
    static_assert(std::is_floating_point_v<T>,
                 "RadToDeg requires floating point type");
    return radians * static_cast<T>(180) / std::numbers::pi_v<T>;
}


// ----------------------------
// Precision Control
// ----------------------------
namespace precision 
{
    template<typename T>
    struct Epsilon {
        static_assert(std::is_floating_point_v<T>,
                     "Epsilon requires floating point type");

        // 基础精度级别
        static constexpr T low()       { return T(1e-3); }
        static constexpr T medium()    { return T(1e-6); }
        static constexpr T high()      { return T(1e-9); }
        static constexpr T machine()   { return std::numeric_limits<T>::epsilon(); }
        
        // 自适应相对精度
        static constexpr T relative(T value) { 
            return std::max(medium(), std::abs(value) * machine());
        }
    };
}


// ----------------------------
// Floating Point Comparisons 
// ----------------------------
template<typename T>
bool is_equal(T a, T b, 
              T epsilon = precision::Epsilon<T>::medium()) noexcept 
{
    // Handle NaN case
    if (std::isnan(a)) return std::isnan(b);
    if (std::isnan(b)) return false;
    
    // Handle infinity case
    if (std::isinf(a) || std::isinf(b)) 
        return a == b;
    
    // Mixed absolute and relative error comparison
    const T diff = std::abs(a - b);
    if (diff <= epsilon) return true;
    
    // Use relative error when values are large
    return diff <= precision::Epsilon<T>::relative(std::max(std::abs(a), std::abs(b)));
}

template<typename T>
bool is_zero(T val, 
             T epsilon = precision::Epsilon<T>::medium()) noexcept 
{
    return std::abs(val) < epsilon && !std::isinf(val);
}


// ----------------------------
// Value Manipulation
// ----------------------------
template<typename T>
constexpr T Clamp(T value, T min_val, T max_val) noexcept 
{
    return (value < min_val) ? min_val : 
           (value > max_val) ? max_val : value;
}

template<typename T>
T NormalizeAngle(T angle) noexcept 
{
    angle = std::fmod(angle, TwoPi<T>());
    if (angle < -Pi<T>()) angle += TwoPi<T>();
    if (angle >= Pi<T>()) angle -= TwoPi<T>();
    return angle;
}

} // namespace wisson_math
