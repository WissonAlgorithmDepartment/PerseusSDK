/**
 * @file perseus_robot.h
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.0
 * @date: 2025-09-01
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Defines the PerseusRobot class for controlling a daularm robot via SDK network.
 *
 * This class provides thread-safe access to robot state, control interface, and SDK network connection.
 */
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include "perseuslib/common/robot_state.hpp"
#include "perseuslib/controller/controller.h"


namespace wisson_SDK {

/**
 * @class PerseusRobot
 * @brief Provides high-level control interface to a daularm-robot via SDK network.
 *
 * The class maintains a network connection, provides current robot state, and allows motion control.
 * All public members are thread-safe.
 *
 * @anchor o-frame
 * @par Base frame O
 * Located at the center of the robot's base. The z-axis points upwards, orthogonal to the horizontal plane.
 *
 * @anchor ee-frame
 * @par End effector frame EE
 * By default, EE coincides with the nominal end effector frame NE (identity transformation). 
 * It can be customized for tools or attachments frequently changed during operation.
 */
class PerseusRobot : public std::enable_shared_from_this<PerseusRobot> 
{
public:
    using ServerVersion = uint32_t; ///< Type for representing robot server software version

    /**
     * @brief Constructs a PerseusRobot instance and initializes SDK network connection.
     * @param config_path Path to configuration file for the robot.
     */
    explicit PerseusRobot(const std::string& config_path);

    /**
     * @brief Move constructor.
     * @param other Another PerseusRobot instance.
     */
    PerseusRobot(PerseusRobot&& other) noexcept;

    /**
     * @brief Move assignment operator.
     * @param other Another PerseusRobot instance.
     * @return Reference to this instance.
     */
    PerseusRobot& operator=(PerseusRobot&& other) noexcept;

    /**
     * @brief Destructor closes network connection.
     */
    virtual ~PerseusRobot() noexcept;

    /**
     * @brief Factory method to create a PerseusRobot instance.
     * @param config_path Configuration file path.
     * @return Shared pointer to the new PerseusRobot instance.
     */
    static std::shared_ptr<PerseusRobot> Create(const std::string& config_path); 

    /**
     * @brief Attempts to establish a hardware connection via SDK network.
     * @return true if connection succeeds, false otherwise.
     * @throw NetworkException if connection fails.
     */
    bool HardwareConnect();

    // /**
    // * Stops all currently running motions.
    // *
    // * If a control or motion generator loop is running in another thread, it will be preempted
    // * with a perseus::ControlException.
    // *
    // * @throw CommandException if the Control reports an error.
    // * @throw NetworkException if the connection is lost, e.g. after a timeout.
    // */
    // void Stop();

    /**
     * @brief Sends a motion command to the robot.
     * @param controller_mode Controller mode (joint/task/etc.)
     * @param cmd RobotCommand object containing target motion.
     */
    void Control(const control::ControllerMode& controller_mode, std::shared_ptr<control::RobotCommand> cmd);

    /**
     * @brief Reads a single robot state update.
     * @return Shared pointer to current RobotState.
     */
    [[nodiscard]] virtual std::shared_ptr<RobotState> ReadOnce();

    /**
     * @brief Returns the connected robot server software version.
     * @return ServerVersion value.
     */
    [[nodiscard]] ServerVersion getServerVersion() const noexcept;

    /**
     * @brief Sets a custom log tag for logging.
     * @param tag Log tag string.
     */
    void SetLogTag(const std::string& tag) { log_tag_ = tag; }

    /// Disable copy constructor and copy assignment
    PerseusRobot(const PerseusRobot&) = delete;
    PerseusRobot& operator=(const PerseusRobot&) = delete;

    class Impl; ///< Forward declaration for PImpl pattern

protected:
    /**
     * @brief Protected constructor used for unit testing with Impl mocks.
     * @param robot_impl Shared pointer to Impl instance.
     */
    PerseusRobot(std::shared_ptr<Impl> robot_impl);

private:
    std::shared_ptr<Impl> impl_;       ///< Implementation detail (PImpl)
    mutable std::mutex control_mutex_; ///< Mutex protecting control commands
    std::string log_tag_;              ///< Custom tag used in logging
};

}  // namespace wisson_SDK 