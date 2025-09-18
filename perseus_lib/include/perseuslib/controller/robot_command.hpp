/**
 * @file robot_command.hpp
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.2
 * @date: 2025-09-02
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Defines command structures, response statuses, refusal reasons,
 *        and utility functions for robot control in libperseus.
 */
#pragma once

#include <array>
#include <chrono>
#include <atomic>
#include <vector>
#include <memory>
#include <string>
#include <string_view>
#include <variant>
#include <stdexcept>

#include "perseuslib/common/robot_state.hpp"


namespace wisson_SDK::control {

// -----------------------------------------------------------------------------------
//                                Constants
// -----------------------------------------------------------------------------------

/**
 * @brief Maximum number of commands in the command list.
 */
inline constexpr std::size_t cmd_list_size = 20;  



// -----------------------------------------------------------------------------------
//                               Command Types                             
// -----------------------------------------------------------------------------------

/**
 * @brief Represents a motion command for the robot.
 *
 * A motion command may specify joint positions, velocities, end-effector transforms,
 * and optional elbow configuration.
 */
struct MotionCommand 
{
private:
    // Private constructor to prevent direct instantiation from outside
    MotionCommand() = default;

public:
    std::array<double, JOINT_NUM> joint_positions{};   ///< Target joint positions [rad].
    std::array<double, JOINT_NUM> joint_velocities{};  ///< Target joint velocities [rad/s].
    std::array<double, 16> ee_transform{};             ///< End-effector homogeneous transform (4x4 matrix, row-major).
    std::array<double, 6> ee_velocity{};               ///< End-effector velocity (linear + angular).
    std::array<double, 2> elbow{};                     ///< Optional elbow configuration.
    bool has_elbow{false};                             ///< Flag indicating if elbow config is provided.
    double timeout{10.0};                              ///< Command timeout in second

    /**
     * @brief Factory method to create a motion command with joint positions.
     * @param joint_positions Desired joint positions.
     * @param timeout_s Timeout in seconds (default: 30.0).
     * @return Constructed MotionCommand.
     */
    static MotionCommand CreateCommand(const std::array<double, JOINT_NUM>& joint_positions,
                                       double timeout_s = 30.0)
    {
        MotionCommand cmd;
        cmd.joint_positions = joint_positions;
        cmd.timeout = timeout_s;
        return cmd;
    }
};


/**
 * @brief Represents a torque command for the robot.
 */
struct TorqueCommand  
{
    std::array<double, JOINT_NUM> desired_torque{}; ///< Desired joint torques [Nm].
    double timeout{10.0};                           ///< Command timeout in seconds.
};


/**
 * @brief Supported end-effector actions.
 */
enum class EndEffectorAction : uint32_t 
{
    Idle = 0,    ///< No action.
    Open,        ///< Open gripper.
    Close,       ///< Close gripper.
    ForceClose   ///< Force close gripper.
};


/**
 * @brief Represents an end-effector command.
 */
struct EndEffectorCommand  
{
    EndEffectorAction ee_action{EndEffectorAction::Idle}; ///< Desired end-effector action.
    double timeout{10.0};                                 ///< Command timeout in seconds. 
};



// -----------------------------------------------------------------------------------
//                              Command Response
// -----------------------------------------------------------------------------------

/**
 * @brief Status of a command execution.
 */
enum class ResponseStatus : uint32_t
{
    kIdle = 0,     ///< Idle, no command in progress.
    kSending,      ///< Command is being sent.
    kWaiting,      ///< Waiting for a response.
    kSubSuccess,   ///< Intermediate success (e.g., waypoint reached).
    kSuccess,      ///< Command completed successfully.
    kFail,         ///< Command failed.
    kUserStop,     ///< Command stopped by user.
    kTimeout,      ///< Command execution timed out.
    kAbort,        ///< Command aborted by the system.
    kRefused,      ///< Command refused by the system.
    kUnknown       ///< Unknown status.
};


/**
 * @brief Reasons why a command may be refused.
 */
enum class RefusedReason : uint32_t
{
    None = 0,                 // No refusal reason
    InvalidRequest = 1,       // Client sent an invalid request
    Unauthorized = 2,         // Client is unauthorized
    NotFound = 3,             // Requested resource not found
    ServerError = 4,          // Internal server error
    Timeout = 5,              // Request timed out
    WrongRequestSource = 6,   // Request came from a wrong source
    SelfCheckInProgress = 7,  // Robot has not completed self-check
    RobotBusy = 8,            // Robot is already running a task
    RobotDismatch = 9         // Robot is not the target device
};



// -----------------------------------------------------------------------------------
//                                   Utils
// -----------------------------------------------------------------------------------
namespace detail {

/**
 * @brief Convert EndEffectorAction enum to string.
 */
[[nodiscard]] inline constexpr std::string_view EndEffectorActionToString(EndEffectorAction action) noexcept
{
    switch (action)
    {
        case EndEffectorAction::Idle:       return "Idle";
        case EndEffectorAction::Open:       return "Open";
        case EndEffectorAction::Close:      return "Close";
        case EndEffectorAction::ForceClose: return "ForceClose";
        default:                            return "Unknown";
    }
}

/**
 * @brief Convert string to EndEffectorAction safely.
 * 
 * If the input string is not recognized, returns Idle as a safe fallback.
 */
[[nodiscard]] inline constexpr EndEffectorAction StringToEndEffectorActionSafe(std::string_view str) noexcept
{
    if (str == "Idle")       return EndEffectorAction::Idle;
    if (str == "Open")       return EndEffectorAction::Open;
    if (str == "Close")      return EndEffectorAction::Close;
    if (str == "ForceClose") return EndEffectorAction::ForceClose;
    return EndEffectorAction::Idle;
}

[[nodiscard]] inline EndEffectorAction StringToEndEffectorActionSafe(const std::string& str) noexcept
{
    return StringToEndEffectorActionSafe(std::string_view{str});
}

[[nodiscard]] inline EndEffectorAction StringToEndEffectorActionSafe(const char* str) noexcept
{
    if (!str) return EndEffectorAction::Idle;
    return StringToEndEffectorActionSafe(std::string_view{str});
}

/**
 * @brief Convert raw integer to ResponseStatus safely.
 *
 * @param result Raw status code.
 * @return Corresponding ResponseStatus, or kUnknown if invalid.
 */
[[nodiscard]] inline constexpr ResponseStatus ToResponseStatus(uint32_t result) noexcept 
{
    using RS = ResponseStatus;
    return (result >= static_cast<uint32_t>(RS::kWaiting) &&
            result <= static_cast<uint32_t>(RS::kRefused))
                ? static_cast<RS>(result)
                : RS::kUnknown;
}

/**
 * @brief Check if a given response status indicates command completion.
 */
[[nodiscard]] inline constexpr bool IsActionFinished(ResponseStatus status) noexcept
{
    switch (status) {
        case ResponseStatus::kSuccess:
        case ResponseStatus::kUserStop:
        case ResponseStatus::kTimeout:
        case ResponseStatus::kAbort:
        case ResponseStatus::kFail:
        case ResponseStatus::kRefused:
            return true;
        default:
            return false;
    }
}

/**
 * @brief Convert ResponseStatus enum to string.
 */
[[nodiscard]] inline constexpr std::string_view ResponseStatusToString(ResponseStatus status) noexcept
{
    switch (status) {
        case ResponseStatus::kIdle:       return "Idle";
        case ResponseStatus::kSending:    return "Sending";
        case ResponseStatus::kWaiting:    return "Waiting";
        case ResponseStatus::kSubSuccess: return "Step Successful";
        case ResponseStatus::kSuccess:    return "Action Completed";
        case ResponseStatus::kFail:       return "Fail";
        case ResponseStatus::kUserStop:   return "User-Stop";
        case ResponseStatus::kTimeout:    return "Timeout";
        case ResponseStatus::kAbort:      return "Abort";
        case ResponseStatus::kRefused:    return "Command Refused";
        case ResponseStatus::kUnknown:
        default:                          return "Unknown";
    }
}

/**
 * @brief Convert RefusedReason enum to string.
 */
[[nodiscard]] inline constexpr std::string_view RefusedReasonToString(RefusedReason reason) noexcept
{
    switch (reason)
    {
        case RefusedReason::None:                return "None";
        case RefusedReason::InvalidRequest:      return "InvalidRequest";
        case RefusedReason::Unauthorized:        return "Unauthorized";
        case RefusedReason::NotFound:            return "NotFound";
        case RefusedReason::ServerError:         return "ServerError";
        case RefusedReason::Timeout:             return "Timeout";
        case RefusedReason::WrongRequestSource:  return "WrongRequestSource";
        case RefusedReason::SelfCheckInProgress: return "SelfCheckInProgress";
        case RefusedReason::RobotBusy:           return "RobotBusy";
        case RefusedReason::RobotDismatch:       return "RobotDismatch";
        default:                                 return "Unknown";
    }
}

} // namespace detail



// -----------------------------------------------------------------------------------
//                                RobotCommand
// -----------------------------------------------------------------------------------
using SDKCmdVariant = std::variant<MotionCommand, TorqueCommand, EndEffectorCommand>;

struct alignas(16) RobotCommand 
{
private:
    // Private constructor to prevent direct instantiation from outside
    RobotCommand() = default;

public:
    uint32_t cmd_id{0};
    size_t cmd_size{0};
    std::vector<SDKCmdVariant> commands;
    double total_timeout{30.0}; 

    /* Command Status */
    std::atomic<size_t> current_index{0};
    std::atomic<bool> finished = false;
    ResponseStatus status{ResponseStatus::kIdle};  

    template <typename CommandType, typename... Args>
    static std::shared_ptr<RobotCommand> CreateCommands(const std::vector<CommandType>& sequence,
                                                        double total_timeout_s = 30.0)
    {
        const size_t N = sequence.size();
        if (N == 0 || N > cmd_list_size) {
            throw wisson_SDK::ConstructorException("libperseus-RobotCommand: Input command vectors are incorrect.");
        }

        auto cmd = std::shared_ptr<RobotCommand>(new RobotCommand());
        cmd->cmd_size = N;
        cmd->total_timeout = total_timeout_s;
        cmd->commands.reserve(N);

        for (size_t i = 0; i < N; ++i) {
            auto c = sequence[i];
            cmd->commands.emplace_back(c);
        }
        return cmd;
    }

    template <typename CommandType>
    static std::shared_ptr<RobotCommand> CreateCommand(const CommandType& c)
    {
        auto cmd = std::shared_ptr<RobotCommand>(new RobotCommand());
        cmd->cmd_size = 1;
        cmd->total_timeout = c.timeout;
        cmd->commands.emplace_back(c);
        return cmd;
    }

    /* Iteration Helpers */
    bool HasNext() const { return current_index < commands.size(); }

    const SDKCmdVariant& Current() const {
        if (current_index >= commands.size()) {
            throw wisson_SDK::ControlException("libperseus-RobotCommand: current_index out of range");
        }
        return commands[current_index];
    }

    void Advance() {
        if (current_index < commands.size()) {
            ++current_index;
        }
    }

    /* Getter Helpers */
    std::vector<std::array<double, JOINT_NUM>> getJointPositionsVec() const {
        std::vector<std::array<double, JOINT_NUM>> joints;
        joints.reserve(commands.size());

        for (const auto& cmd : commands) {
            if (const auto* m = std::get_if<MotionCommand>(&cmd)) {
                joints.push_back(m->joint_positions);
            }
        }
        return joints;
    }

    std::vector<double> getTimeoutVec() const {
        std::vector<double> timeouts;
        timeouts.reserve(commands.size());

        for (const auto& cmd : commands) {
            std::visit([&](auto&& c) { timeouts.push_back(c.timeout); }, cmd);
        }
        return timeouts;
    }

    std::vector<std::string> getEEActionsVecStr() const {
        std::vector<std::string> actions;
        actions.reserve(commands.size());

        for (const auto& cmd : commands) {
            if (const auto* m = std::get_if<EndEffectorCommand>(&cmd)) {
                actions.push_back(std::string(detail::EndEffectorActionToString(m->ee_action)));
            }
        }
        return actions;
    }
};

}  // namespace wisson_SDK::control