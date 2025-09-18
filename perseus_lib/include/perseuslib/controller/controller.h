/**
 * @file controller.h
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.1
 * @date: 2025-09-02
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Defines the Controller class and related enums for PerseusRobot.
 *
 * This header contains:
 *  - ControlSpace / ControlType enums
 *  - ControllerMode struct
 *  - Controller class interface
 *
 * @note Uses C++20 features like constexpr, [[nodiscard]], defaulted comparison operators.
 */
#pragma once

#include <memory>
#include <functional>
#include <string_view>
#include <string>
#include <atomic>
#include <chrono>

#include "perseuslib/common/robot_state.hpp"
#include "perseuslib/common/wisson_exception.hpp"
#include "robot_command.hpp"


namespace wisson_SDK {
  class TcpClient;
  class SDKNetwork;
}

namespace wisson_SDK::control {

// -----------------------------------------------------------------------------------
//                              Enum Definitions                            
// -----------------------------------------------------------------------------------

/**
 * @brief Defines the space in which robot control is applied.
 */
enum class ControlSpace : uint8_t 
{
  kJoint,       ///< Joint space control
  kCartesian,   ///< Cartesian space control
  kTask,        ///< Task space control
  kNullSpace,   ///< Null-space control
  kUserDefined, ///< Custom control space
  kUnknown      ///< Unknown/undefined control space
};


/**
 * @brief Defines the type of control applied to the robot.
 */
enum class ControlType : uint8_t 
{
  kPosition,   ///< Position control
  kVelocity,   ///< Velocity control
  kTorque,     ///< Torque control
  kImpedance,  ///< Impedance control
  kAdmittance, ///< Admittance control
  kCommand,    ///< Command-based control
  kExtern,     ///< External control
  kUnknown     ///< Unknown/undefined control type 
};


namespace detail {

/**
 * @brief Converts ControlSpace enum to string.
 */
[[nodiscard]] inline constexpr std::string_view ControlSpaceToString(ControlSpace cs) noexcept
{
  switch (cs)
  {
    case ControlSpace::kJoint:       return "Joint";
    case ControlSpace::kCartesian:   return "Cartesian";
    case ControlSpace::kTask:        return "Task";
    case ControlSpace::kNullSpace:   return "NullSpace";
    case ControlSpace::kUserDefined: return "UserDefined";
    case ControlSpace::kUnknown:     [[fallthrough]];
    default:                         return "UnknownSpace";
  }
}


/**
 * @brief Converts ControlType enum to string.
 */
[[nodiscard]] inline constexpr std::string_view ControlTypeToString(ControlType ct) noexcept
{
  switch (ct)
  {
    case ControlType::kPosition:   return "Position";
    case ControlType::kVelocity:   return "Velocity";
    case ControlType::kTorque:     return "Torque";
    case ControlType::kImpedance:  return "Impedance";
    case ControlType::kAdmittance: return "Admittance";
    case ControlType::kCommand:    return "Command";
    case ControlType::kExtern:     return "Extern";
    case ControlType::kUnknown:    [[fallthrough]];
    default:                       return "UnknownType"; 
  }
}

} // namespace detail



// -----------------------------------------------------------------------------------
//                           ControllerMode Struct                            
// -----------------------------------------------------------------------------------

/**
 * @brief Represents a control mode consisting of a control space and type.
 */
struct ControllerMode 
{
  ControlSpace space{ControlSpace::kUnknown};
  ControlType type{ControlType::kUnknown};

  constexpr ControllerMode() = default;
  constexpr ControllerMode(ControlSpace s, ControlType t) : space(s), type(t) {}

  /**
   * @brief Factory method to create a ControllerMode.
   */
  static constexpr ControllerMode Create(ControlSpace s, ControlType t) { return {s, t}; }

  /**
   * @brief Common predefined modes
   */
  static constexpr ControllerMode JointPosition() { return {ControlSpace::kJoint, ControlType::kPosition}; }
  static constexpr ControllerMode TaskCommand()   { return {ControlSpace::kTask,  ControlType::kCommand}; }

  constexpr bool operator==(const ControllerMode&) const = default;
  constexpr bool is(ControlSpace s, ControlType t) const { return space == s && type == t; }
  constexpr bool is(const ControllerMode& other) const { return *this == other; }
  
  /**
   * @brief Converts mode to string for logging or display.
   */
  [[nodiscard]] std::string ModeToString() const
    { return std::string(detail::ControlSpaceToString(space)) + "-" + 
             std::string(detail::ControlTypeToString(type)); }
};



// -----------------------------------------------------------------------------------
//                           Controller Class                            
// -----------------------------------------------------------------------------------

/**
 * @brief Represents a robot controller which executes commands in a specific mode.
 *
 * Provides:
 *  - Thread-safe command ID generation
 *  - Network binding for SDK communication
 *  - Callbacks for command completion/waiting
 */
class Controller : public std::enable_shared_from_this<Controller> 
{
public:
  /**
   * @brief Constructs a Controller with a specified mode.
   * @param mode  Control mode (joint/cartesian, position/velocity, etc.)
   */
  Controller(ControllerMode mode);

  virtual ~Controller() noexcept = default;

  /**
   * @brief Factory method to create and return a shared pointer to a Controller.
   */
  static std::shared_ptr<Controller> Create(ControllerMode mode); 

  /**
   * @brief Execute a robot command in a specified controller mode.
   * @param controller_mode  Desired control mode.
   * @param cmd              Command to execute.
   * @return true if command started successfully.
   */
  bool ExecuteMotion(const ControllerMode& controller_mode, std::shared_ptr<RobotCommand> cmd);

  /**
   * @brief Set a callback for reporting command waiting time.
   */
  void SetWaitingCallback(std::function<void(double timecost)> cb) { waiting_callback_ = std::move(cb); }

  /**
   * @brief Check if the controller is currently executing a command.
   */
  bool IsControllerRunning() const noexcept;

  /**
   * @brief Bind the controller to a network interface for SDK communication.
   */
  bool BindNetwork(const std::shared_ptr<SDKNetwork>& network); 

  /**
   * @brief Generate a unique incrementing command ID (thread-safe).
   */
  static uint32_t GenerateCommandId() {
    static std::atomic<uint32_t> commandId{0};
    return ++commandId;
  }

private:
  bool StartMotion(const ControllerMode& controller_mode, std::shared_ptr<RobotCommand> cmd);
  void FinishMotion(); 
  void SendCommand();

private:
  ControllerMode mode_;
  std::shared_ptr<RobotCommand> cmd_;
  std::function<RobotCommand(const RobotState&)> control_callback_{};
  bool running_{false};
  std::shared_ptr<SDKNetwork> network_;
  std::function<void(double timecost)> waiting_callback_;
  std::string log_tag_;
  std::chrono::steady_clock::time_point action_start_time_;
};

}  // namespace wisson_SDK::control 