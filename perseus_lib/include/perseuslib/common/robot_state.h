/**
 * @file robot_state.h
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.0
 * @date: 2025-09-02
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Contains the perseuslib::RobotState types.
 */
#pragma once

#include <array>
#include <ostream>

namespace wisson_SDK {

inline constexpr std::size_t JOINT_NUM = 9;  
inline constexpr std::size_t CHAMBER_NUM = 18;  

/**
 * Explanation of Air Pressure Unit Conversion:
 *
 * - 1 Pa   = 1       N/m²
 * - 1 hPa* = 100     Pa
 * - 1 kPa  = 1000    Pa
 * - 1 bar  ≈ 10^5    Pa
 * - 1 atm  ≈ 101325  Pa ≈ 1.01325 bar
 * - 1 mmHg ≈ 133.322 Pa
 */

/**
 * Describes the robot's current mode.
 */
enum class RobotMode {
  kIdle,
  kCommandMove,
  kUserStopped
};

/**
 * Describes the robot state.
 */
struct RobotState 
{
  /**
  * \f$q\f$
  * Measured joint positions.
  * Vector of size JOINT_NUM, expressed in Unit: \f$[rad]\f$.
  */
  std::array<double,JOINT_NUM> q{};

  /**
  * \f$q_err\f$
  * Measured joint errors.
  * Vector of size JOINT_NUM, expressed in Unit: \f$[rad]\f$.
  */
  std::array<double,JOINT_NUM> q_err{};

  /**
  * Measured joint pressures.
  * Vector of size CHAMBER_NUM, expressed in unit: \f$[hPa]\f$.
  */
  std::array<int,CHAMBER_NUM> pressure{};

  /**
  * Pressure value at the source side.
  * Expressed in unit: \f$[hPa]\f$.
  */
  int pSource;

  /**
  * Pressure value at the sink side.
  * Expressed in unit: \f$[hPa]\f$.
  */
  int pSink;

  /**
   * \f$m_{total}\f$
   * Sum of the mass of the end effector and the external load.
   */
  double m_total{};

  /**
   * \f$^{O}T_{EE}\f$
   * Measured end effector pose in @ref o-frame "base frame".
   * Pose is represented as a 4x4 matrix in column-major format.
   */
  std::array<double, 16> O_T_EE{};  

  /**
   * Current robot mode.
   */
  RobotMode robot_mode = RobotMode::kUserStopped;

  /**
   * @brief Reset all robot state data to default values.
   */
  void ClearData() 
  {
    q.fill(0.0);                    ///< Joint positions (rad), reset to 0
    q_err.fill(0.0);                ///< Joint errors (rad), reset to 0
    pressure.fill(0);               ///< Pressure sensors (e.g., per joint or gripper), reset to 0
    pSource = 0;                    ///< Source pressure, reset to 0
    pSink = 0;                      ///< Sink pressure, reset to 0
    m_total = 0.0;                  ///< Measured torque/force total (or similar aggregate), reset to 0
    O_T_EE.fill(0.0);               ///< End-effector pose matrix, reset to all zeros
    robot_mode = RobotMode::kIdle;  ///< Robot mode, reset to Idle
  }
};

/**
 * Streams the robot state as JSON object: {"field_name_1": [0,0,0,0,0,0,0], "field_name_2":
 * [0,0,0,0,0,0], ...}
 *
 * @param[in] ostream Ostream instance
 * @param[in] robot_state RobotState instance to stream
 *
 * @return Ostream instance
 */
std::ostream& operator<<(std::ostream& ostream, const wisson_SDK::RobotState& robot_state);

/**
 * Streams RobotMode in human-readable form

 * @param[in] ostream Ostream instance
 * @param[in] robot_mode RobotMode to stream
 *
 * @return Ostream instance
 */
std::ostream& operator<<(std::ostream& ostream, RobotMode robot_mode);

}  // namespace wisson_SDK