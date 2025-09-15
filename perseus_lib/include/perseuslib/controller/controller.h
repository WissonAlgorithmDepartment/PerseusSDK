/**
 * @file controller.h
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.0
 * @date: 2025-09-02
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Contains the perseuslib::PerseusRobot type.
 */
#pragma once

#include <memory>
#include <chrono>
#include <functional>

#include "perseuslib/common/robot_state.h"


namespace wisson_SDK {

inline std::chrono::_V2::steady_clock::time_point TIC(){
  return std::chrono::steady_clock::now();
}
inline double TOC(std::chrono::_V2::steady_clock::time_point start_time_point){
  return (std::chrono::duration_cast<std::chrono::duration<double>>(
              std::chrono::steady_clock::now() - start_time_point)).count();
}

// -----------------------------------------------------------------------------------
//                              Struct Definition                             
// -----------------------------------------------------------------------------------
enum class ControlSpace 
{
  kJoint,     
  // kCartesian,  
  // kTask, 
  // kNullSpace, 
  // kUserDefined
};

enum class ControlType 
{
  kPosition,      
  // kVelocity,      
  // kTorque,        
  // kImpedance,     
  // kAdmittance,
  // kExtern     
};

struct ControllerMode 
{
  ControlSpace space{};
  ControlType type{};

  constexpr ControllerMode(ControlSpace s, ControlType t) : space(s), type(t) {}

  static constexpr ControllerMode JointPosition() { return {ControlSpace::kJoint, ControlType::kPosition}; }

  bool operator==(const ControllerMode& other) const {
    return space == other.space && type == other.type;
  }

  bool operator!=(const ControllerMode& other) const {
    return !(*this == other);
  }
};

struct MotionCommand 
{
  std::array<double, JOINT_NUM> joint_positions{};      
  std::array<double, JOINT_NUM> joint_velocities{};     
  std::array<double, 16> ee_transform{};          
  std::array<double, 6> ee_velocity{};        
  std::array<double, 2> elbow{};                
  bool has_elbow = false;                             
};

struct TorqueCommand  
{
  std::array<double, JOINT_NUM> desired_torque{};
};

enum class ResponseStatus : uint32_t
{
  kIdle = 0,
  kSending,
  kWaiting,
  kSuccess,
  kFail,      
  kUserStop,      
  kTimeout,        
  kAbort, 
  kRefused,    
  kUnknown   
};

namespace detail {

inline ResponseStatus ToResponseStatus(uint32_t result) 
{
  using RS = ResponseStatus;
  return (result >= static_cast<uint32_t>(RS::kWaiting) &&
          result <= static_cast<uint32_t>(RS::kRefused))
              ? static_cast<RS>(result)
              : RS::kUnknown;
}

// Check whether the state indicates the action has finished
inline bool IsActionFinished(ResponseStatus status)
{
  switch (status) {
    case ResponseStatus::kSuccess:
    case ResponseStatus::kUserStop:
    case ResponseStatus::kTimeout:
    case ResponseStatus::kAbort:
    case ResponseStatus::kFail:
    case ResponseStatus::kRefused:
      return true;   // These states indicate that the action has already finished
    case ResponseStatus::kIdle:
    case ResponseStatus::kSending:
    case ResponseStatus::kWaiting:
    case ResponseStatus::kUnknown:
    default:
      return false;  // The action has not finished yet or its status is unknown
  }
}

inline std::string RespStatusToString(ResponseStatus status)
{
  switch (status) {
    case ResponseStatus::kIdle:      return "Idle";
    case ResponseStatus::kSending:   return "Sending";
    case ResponseStatus::kWaiting:   return "Waiting";
    case ResponseStatus::kSuccess:   return "Successful";
    case ResponseStatus::kFail:      return "Fail";
    case ResponseStatus::kUserStop:  return "User-Stop";
    case ResponseStatus::kTimeout:   return "Timeout";
    case ResponseStatus::kAbort:     return "Abort";
    case ResponseStatus::kRefused:   return "Coomand Refused";
    case ResponseStatus::kUnknown:   return "Unknown";
    default:                         return "Invalid-Status";
  }
}

} // namespace detail

struct RobotCommand 
{
private:
  // Private constructor to prevent direct instantiation from outside
  RobotCommand() = default;

public:
  uint32_t cmd_id{0};
  MotionCommand motion{};
  TorqueCommand control{};
  double timeout{30.0}; 

  // Command Status
  std::atomic<bool> finished = false;
  ResponseStatus status = ResponseStatus::kIdle; 
  
  static std::unique_ptr<RobotCommand> JointPosition(const std::array<double, JOINT_NUM>& desired_joint,
                                                     double timeout_s = 30.0) 
  { 
    std::unique_ptr<RobotCommand> cmd(new RobotCommand());
    cmd->motion.joint_positions = desired_joint;
    cmd->timeout = timeout_s;
    return cmd; 
  }
};


class TcpClient;
class SDKNetwork;

// -----------------------------------------------------------------------------------
//                           Controller Definition                             
// -----------------------------------------------------------------------------------
class Controller : public std::enable_shared_from_this<Controller> 
{
public:
  /**
    * @brief Construct a RobotController with a given control mode.
    * @param mode  Control mode (joint/cartesian, position/velocity).
    */
  Controller(ControllerMode mode);

  virtual ~Controller() noexcept = default;

  /**
  * Creates a new Controller instance and returns a shared pointer to it.
  *
  * @return Shared pointer to the newly created Controller instance.
  */
  static std::shared_ptr<Controller> Create(ControllerMode mode); 

  bool ExecuteMotion(const ControllerMode& controller_mode, std::unique_ptr<RobotCommand> cmd);

  void SetWaitingCallback(std::function<void(double timecost)> cb) { waiting_callback_ = std::move(cb); }

  bool IsControllerRunning() const noexcept;

  bool BindNetwork(const std::shared_ptr<SDKNetwork>& network); 

  // Generate a unique incrementing command ID (thread-safe)
  static uint32_t GenerateCommandId() {
    static std::atomic<uint32_t> commandId{0};
    return ++commandId;
  }

private:
  bool StartMotion(const ControllerMode& controller_mode, std::unique_ptr<RobotCommand> cmd);
  void FinishMotion(); 
  void SendCommand();

  ControllerMode mode_;
  std::unique_ptr<RobotCommand> cmd_;

  std::function<RobotCommand(const RobotState&)> control_callback_{};

  bool running_{false};

  std::shared_ptr<SDKNetwork> network_;

  std::function<void(double timecost)> waiting_callback_;
};

}  // namespace wisson_SDK 