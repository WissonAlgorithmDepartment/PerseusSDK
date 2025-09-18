/**
 * @file timer_utils.hpp
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.0
 * @date: 2025-09-16
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Utility functions for simple timing measurements (TIC/TOC style).
 * @example:
 *   auto t0 = TIC();
 *   // do something...
 *   double elapsed = TOC(t0); // elapsed time in seconds
 */
#pragma once

#include <chrono>


namespace wisson_SDK::timer {

/**
 * @brief Get current steady clock time point.
 * 
 * @return std::chrono::steady_clock::time_point 
 */
inline std::chrono::_V2::steady_clock::time_point TIC(){
  return std::chrono::steady_clock::now();
}


/**
 * @brief Compute elapsed time in seconds since the given start time point.
 * 
 * @param start_time_point The start time point returned by TIC().
 * @return double Elapsed time in seconds.
 */
inline double TOC(std::chrono::_V2::steady_clock::time_point start_time_point){
  return (std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - start_time_point)).count();
}

} // namespace wisson_SDK::timer