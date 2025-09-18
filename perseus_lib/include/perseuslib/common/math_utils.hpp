/**
 * @file math_utils.hpp
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.3
 * @date: 2025-05-07
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Enhanced mathematics utilities for robotics applications.
 *
 * This header provides a small, safe, and well-documented set of floating-point
 * utilities commonly used in robotics:
 *  - precise Pi / angle constants (templated on floating point type)
 *  - unit conversions between degrees and radians
 *  - precision / epsilon helpers
 *  - robust floating point comparisons (absolute + relative)
 *  - clamp and normalize-angle helpers
 *
 * Requirements: C++20 (for concepts and std::numbers). Designed to be header-only.
 *
 */
#pragma once

#include <algorithm>    // std::clamp, std::max
#include <cmath>        // std::fmod, std::fabs, std::isnan, std::isinf
#include <limits>
#include <numbers>      // std::numbers::pi_v
#include <type_traits>
#include <concepts>     // std::floating_point
#include <cstdint>


namespace wisson_SDK::math {

// -----------------------------------------------------------------------------------
//                              Type Aliases                             
// -----------------------------------------------------------------------------------
using float32_t = float;
using float64_t = double;



// -----------------------------------------------------------------------------------
//                    Mathematical constants (templated)                            
// -----------------------------------------------------------------------------------
/**
 * @brief Return value of π for floating point type T.
 * @tparam T floating-point type (constrained)
 * @return pi as T
 */
template<std::floating_point T>
[[nodiscard]] constexpr T Pi() noexcept
{
    return std::numbers::pi_v<T>;
}

/** @brief 2π for floating point type T */
template<std::floating_point T>
[[nodiscard]] constexpr T TwoPi() noexcept
{
    return static_cast<T>(2) * std::numbers::pi_v<T>;
}

/** @brief π/2 for floating point type T */
template<std::floating_point T>
[[nodiscard]] constexpr T HalfPi() noexcept
{
    return std::numbers::pi_v<T> / static_cast<T>(2);
}



// -----------------------------------------------------------------------------------
//                    Degree/radian conversions                            
// -----------------------------------------------------------------------------------
/**
 * @brief Convert degrees to radians.
 * @tparam T floating-point type
 * @param degrees angle in degrees
 * @return angle in radians
 */
template<std::floating_point T>
[[nodiscard]] constexpr T deg_to_rad(T degrees) noexcept
{
    return degrees * (std::numbers::pi_v<T> / static_cast<T>(180));
}

/**
 * @brief Convert radians to degrees.
 * @tparam T floating-point type
 * @param radians angle in radians
 * @return angle in degrees
 */
template<std::floating_point T>
[[nodiscard]] constexpr T rad_to_deg(T radians) noexcept
{
    return radians * (static_cast<T>(180) / std::numbers::pi_v<T>);
}

// Small convenience constants for double precision (legacy compatibility)
inline constexpr double DEG_1_IN_RAD   = deg_to_rad<double>(1.0);
inline constexpr double DEG_0_1_IN_RAD = deg_to_rad<double>(0.1);
// constexpr double DEG_1_IN_RAD   = 0.0174532925199;
// constexpr double DEG_0_1_IN_RAD = 0.00174532925199;



// -----------------------------------------------------------------------------------
//                    Precision control helpers                            
// -----------------------------------------------------------------------------------
namespace precision {

/**
 * @brief Epsilon levels for floating point comparisons.
 * @tparam T floating-point type
 */
template<std::floating_point T>
struct Epsilon
{
    static constexpr T low()    noexcept { return static_cast<T>(1e-3); }
    static constexpr T medium() noexcept { return static_cast<T>(1e-6); }
    static constexpr T high()   noexcept { return static_cast<T>(1e-9); }
    static constexpr T machine()noexcept { return std::numeric_limits<T>::epsilon(); }

    /**
     * @brief Relative epsilon scaled by magnitude of value.
     * @param value value used to compute relative tolerance
     * @return scaled relative tolerance
     *
     * Use for comparisons when values can be large.
     */
    static constexpr T relative(T value) noexcept
    {
        // max between a medium absolute threshold and machine * |value|
        const T scaled = static_cast<T>(std::fabs(value)) * machine();
        return std::max(medium(), scaled);
    }
};

} // namespace precision



// -----------------------------------------------------------------------------------
//                    Floating-point comparisons                            
// -----------------------------------------------------------------------------------
/**
 * @brief Robust floating-point equality: handles NaN, infinities, absolute and relative tolerances.
 *
 * Behavior:
 *  - If a is NaN, returns true iff b is NaN.
 *  - If either is infinite, compares for exact equality.
 *  - Uses absolute epsilon first; if difference is larger, falls back to relative tolerance.
 *
 * @tparam T floating-point type
 * @param a first value
 * @param b second value
 * @param epsilon absolute tolerance to use first (defaults to medium)
 * @return true if values considered equal
 */
template<std::floating_point T>
[[nodiscard]] inline bool is_equal(T a, T b, T epsilon = precision::Epsilon<T>::medium()) noexcept
{
    // NaN handling
    if (std::isnan(a)) return std::isnan(b);
    if (std::isnan(b)) return false;

    // Infinity handling
    if (std::isinf(a) || std::isinf(b)) return a == b;

    const T diff = static_cast<T>(std::fabs(a - b));
    if (diff <= epsilon) return true;

    // relative comparison using magnitude of the larger value
    const T max_abs = std::max(static_cast<T>(std::fabs(a)), static_cast<T>(std::fabs(b)));
    return diff <= precision::Epsilon<T>::relative(max_abs);
}

/**
 * @brief Test whether a floating point value is (near) zero.
 * @tparam T floating-point type
 * @param val value to test
 * @param epsilon tolerance (default medium)
 * @return true if |val| < epsilon and val is not infinite
 */
template<std::floating_point T>
[[nodiscard]] inline bool is_zero(T val, T epsilon = precision::Epsilon<T>::medium()) noexcept
{
    return (std::fabs(val) < epsilon) && !std::isinf(val);
}


// -----------------------------------------------------------------------------------
//                    Value manipulation helpers                            
// -----------------------------------------------------------------------------------
/**
 * @brief Clamp a value between min_val and max_val.
 * @tparam T type supporting comparison
 * @param value value to clamp
 * @param min_val lower bound
 * @param max_val upper bound
 * @return clamped value
 */
template<typename T>
[[nodiscard]] constexpr T Clamp(T value, T min_val, T max_val) noexcept
{
    // Prefer std::clamp when available, but keep constexpr compatibility
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief Normalize angle to range [-pi, pi).
 * @tparam T floating-point type
 * @param angle angle in radians
 * @return normalized angle in [-pi, pi)
 *
 * This function is robust to large-magnitude inputs by using fmod and then
 * shifting into the canonical range.
 */
template<std::floating_point T>
[[nodiscard]] inline T NormalizeAngle(T angle) noexcept
{
    const T two_pi = TwoPi<T>();
    // fmod can return negative remainders; reduce first
    angle = static_cast<T>(std::fmod(angle, two_pi));
    if (angle < -Pi<T>()) angle += two_pi;
    else if (angle >= Pi<T>()) angle -= two_pi;
    return angle;
}

} // namespace wisson_SDK::math