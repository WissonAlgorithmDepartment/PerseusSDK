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

#include <sstream>
#include <string>
#include <iomanip>  // for std::setprecision, std::setw
#include <cmath>    // for std::abs
#include <array>
#include <vector>
#include <type_traits>
#include <spdlog/spdlog.h>

#include "perseuslib/common/math_common.hpp"

namespace wisson_SDK::logging {

// // -----------------------------------------------------------------------------------
// //                               Matrix                             
// // -----------------------------------------------------------------------------------
// /**
//  * @brief Convert an Eigen::Matrix4d to a formatted string for logging.
//  *
//  * @format:
//  * - Scientific notation with 3 decimal places
//  * - Values close to 0 or 1 replaced by exact "0" or "1" for readability
//  * - Each row is printed with aligned columns and enclosed in square brackets
//  * - Total width per value is 12 characters
//  *
//  * Example output:
//  * [            1            0            0            0 ]
//  * [            0            1            0            0 ]
//  * [            0            0            1            0 ]
//  * [            0            0            0            1 ]
//  *
//  * @param mat 4x4 matrix to convert
//  * @return std::string Formatted string
//  */
// inline std::string matrix4d_to_string(const Eigen::Matrix4d& mat) 
// {
//     // Solution 1:
//     // Eigen::IOFormat format(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n  ", "[", "]");
//     // std::stringstream ss;
//     // ss << mat.format(format);
//     // return ss.str();


//     // Solution 2:
//     // std::stringstream ss;
//     // // 使用科学计数法，保留3位小数，固定宽度对齐
//     // ss << std::scientific << std::setprecision(3);
    
//     // for (int i = 0; i < 4; ++i) {
//     //     ss << "[ ";
//     //     for (int j = 0; j < 4; ++j) {
//     //         // 每个元素占12字符宽度，右对齐
//     //         ss << std::setw(12) << mat(i, j);
//     //         if (j < 3) ss << " "; // 列间分隔
//     //     }
//     //     ss << " ]"; // 行结束
//     //     if (i < 3) ss << "\n  "; // 行间分隔（最后一行不加）
//     // }
//     // return ss.str();


//     // Solution 3:
//     std::stringstream ss;
//     ss << std::scientific << std::setprecision(3);

//     for (int i = 0; i < 4; ++i) {
//         ss << "[ ";
//         for (int j = 0; j < 4; ++j) {
//             const double val = mat(i, j);
//             if (wisson_math::is_zero(val)) {
//                 ss << std::setw(12) << "0";               // 占12字符宽度，右对齐
//             // } else if (std::abs(val - 1.0) < wisson_math::M_EPSILON) {
//             } else if (wisson_math::is_equal(val, 1.0)) {
//                 ss << std::setw(12) << "1";
//             // } else if (std::abs(val + 1.0) < wisson_math::M_EPSILON) {
//             } else if (wisson_math::is_equal(val, -1.0)) {
//                 ss << std::setw(12) << "-1";
//             } else {
//                 ss << std::setw(12) << val;               // 其他值保持科学计数法
//             }
//             if (j < 3) ss << " ";                         // 列间分隔
//         }
//         ss << " ]";                                       // 行尾
//         if (i < 3) ss << "\n  ";                          // 行间分隔（最后一行不加）
//     }
//     return ss.str();
// }


// -----------------------------------------------------------------------------------
//                      STL For Degrees or Radians                             
// -----------------------------------------------------------------------------------
enum class AngleUnit { Radians, Degrees };

template<typename Container>
inline std::string format_perseus_angles(const Container& data, AngleUnit source_unit = AngleUnit::Radians,
                                        AngleUnit target_unit = AngleUnit::Degrees,
                                        int precision = 4, bool show_unit_symbol = true) noexcept 
{
    // 类型检查
    using ValueType = typename Container::value_type;
    static_assert(std::is_floating_point_v<ValueType>,
                 "Container must hold floating-point values");
    static_assert(std::is_same_v<Container, std::vector<ValueType>> ||
                 std::is_same_v<Container, std::array<ValueType, std::tuple_size_v<Container>>>,
                 "Unsupported container type");
    
    // 空容器处理
    if (data.empty()) return "[Empty]";

    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed;
    oss << "[";
    oss << *data.begin() << "m";
    oss.precision(precision);

    // 单位符号
    const char* unit_symbol = "";
    if (show_unit_symbol) {
        unit_symbol = (target_unit == AngleUnit::Degrees) ? "\u00B0" : "rad";
    }

   // 预处理转换函数 (避免循环内重复判断)
    const auto converter = [source_unit, target_unit](ValueType val) {
        if (source_unit == target_unit) return val;
        
        return (target_unit == AngleUnit::Degrees) 
            ? wisson_math::rad_to_deg(val)
            : wisson_math::deg_to_rad(val);
    };

    // 格式化输出优化
    for (auto it = std::next(data.begin()); it != data.end(); ++it) {
        if (it != data.begin()) {
            oss << ", ";
        }
        
        // 数值转换
        const ValueType converted = converter(*it);
        
        // 处理异常值
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


// 性能监控结构体
struct AsyncMonitor {
    std::atomic<uint64_t> total_bytes{0};
    std::atomic<uint32_t> drop_count{0};
    std::atomic<uint32_t> log_count{0};
    std::chrono::steady_clock::time_point start;
};


inline std::string shorten_portname(const std::string& port) 
{
    // Bsp: 从 "/tmp/ttyV1" 提取 "ttyV1"
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

} // namespace wisson_SDK::logging