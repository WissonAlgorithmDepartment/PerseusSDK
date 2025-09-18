/**
 * @file log_common.hpp
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.0
 * @date: 2025-05-08
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Logging utilities with customized formatting for Eigen matrix types and other structures.
 * 
 * This header file contains logging utility functions designed to log Eigen matrices and other
 * data types with custom formatting. The functions allow easy integration with the logging
 * framework and help format output for matrices or vectors in a human-readable form.
 */
#pragma once

// ------------------- Standard Library -------------------
#include <array>
#include <atomic>
#include <cmath>            // std::isnan, std::isinf
#include <iomanip>
#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include <chrono>

// ------------------- Third-Party Library ----------------
#include <spdlog/spdlog.h>
#include <fmt/format.h>

// ------------------- Local Headers ----------------------
#include "perseuslib/common/math_utils.hpp"


namespace wisson_SDK::logging {

inline constexpr char INDENT4[] = "    ";

// -----------------------------------------------------------------------------------
//                      STL For Degrees or Radians                             
// -----------------------------------------------------------------------------------
enum class AngleUnit { Radians, Degrees };

template<typename Container>
inline std::string format_perseus_angles(const Container& data, AngleUnit source_unit = AngleUnit::Radians,
                                        AngleUnit target_unit = AngleUnit::Degrees,
                                        int precision = 4, bool show_unit_symbol = true) noexcept 
{
    // Type checking
    using ValueType = typename Container::value_type;
    static_assert(std::is_floating_point_v<ValueType>,
                 "Container must hold floating-point values");
    static_assert(std::is_same_v<Container, std::vector<ValueType>> ||
                 std::is_same_v<Container, std::array<ValueType, std::tuple_size_v<Container>>>,
                 "Unsupported container type");
    
    // Handle empty containers
    if (data.empty()) return "[Empty]";

    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed;
    oss << "[";
    oss << *data.begin() << "m";
    oss.precision(precision);

    // Unit symbols
    const char* unit_symbol = "";
    if (show_unit_symbol) {
        unit_symbol = (target_unit == AngleUnit::Degrees) ? "\u00B0" : "rad";
    }

   // Preprocessing conversion function (avoid repeated checks inside loops)
    const auto converter = [source_unit, target_unit](ValueType val) {
        if (source_unit == target_unit) return val;
        
        return (target_unit == AngleUnit::Degrees) 
            ? math::rad_to_deg(val)
            : math::deg_to_rad(val);
    };

    // Optimized formatted output
    for (auto it = std::next(data.begin()); it != data.end(); ++it) {
        if (it != data.begin()) {
            oss << ", ";
        }
        
        // Numeric conversion
        const ValueType converted = converter(*it);
        
        // Handle abnormal values
        if (std::isnan(converted)) {
            oss << "NaN";
        } else if (std::isinf(converted)) {
            oss << ((converted > 0) ? "+Inf" : "-Inf");
        } else {
            oss << converted;
        }
        
        oss << unit_symbol;
    }
    oss << "]";

    return oss.str();
}


// Performance monitoring struct
struct AsyncMonitor {
    std::atomic<uint64_t> total_bytes{0};
    std::atomic<uint32_t> drop_count{0};
    std::atomic<uint32_t> log_count{0};
    std::chrono::steady_clock::time_point start;
};


inline std::string shorten_portname(const std::string& port) 
{
    // Bsp: extract "ttyV1" from "/tmp/ttyV1"
    auto pos = port.find_last_of('/');
    std::string base = (pos != std::string::npos) ? port.substr(pos + 1) : port;
    return base;  
}


// -----------------------------------------------------------------------------------
//                                Logging Utils                             
// -----------------------------------------------------------------------------------
inline std::string MakeLogTag(const std::string& main_name, const std::string& function_name) {
    return fmt::format("[{}] [{}]", main_name, function_name);
}


// -----------------------------------------------------------------------------------
//                             RobotCommand Related                             
// -----------------------------------------------------------------------------------
inline std::string PrintMotionCommand(const std::array<double, 9>& joints, double timeout) 
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    oss << INDENT4 << "Joints = [" << joints[0] << "m, ";
    oss << std::fixed << std::setprecision(2);
    for (size_t i = 1; i < 9; ++i) {
        oss << joints[i] << "\u00B0";
        if (i + 1 < 9) oss << ", ";
    }
    oss << std::fixed << std::setprecision(1);
    oss << "], Timeout = [" << timeout << "s]";
    return oss.str();
}


inline std::string PrintEndEffectorCommand(std::string_view cmd, double timeout) 
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << INDENT4 << "EndEffector Command = [" << cmd << "], Timeout = [" << timeout << "s]";
    return oss.str();
}

} // namespace wisson_SDK::logging