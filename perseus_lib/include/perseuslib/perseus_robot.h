/**
 * @file perseus_robot.h
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.0
 * @date: 2025-09-01
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Contains the perseuslib::PerseusRobot type.
 */
#pragma once

#include <memory>
#include <mutex>

#include "perseuslib/common/robot_state.h"
#include "perseuslib/controller/controller.h"


namespace wisson_SDK {

/**
 * Maintains a sdk_network connection to the daularm-robot, provides the current robot state, gives access to
 * the model library and allows to control the robot.
 *
 * @note
 * The members of this class are threadsafe.
 *
 * @anchor o-frame
 * @par Base frame O
 * The base frame is located at the center of the robot's base. The z-axis is established as the 
 * axis orthogonal to the horizontal plane and directed upwards.
 *
 *
 * @anchor ee-frame
 * @par end effector frame EE
 * By default, the end effector frame EE is the same as the nominal end effector frame NE
 * (i.e. the transformation between NE and EE is the identity transformation). It may be used to set
 * end effector frames which are changed more frequently (such as a tool that is grasped with the
 * end effector).
 */
class PerseusRobot : public std::enable_shared_from_this<PerseusRobot> 
{
public:
    /**
    * Version of the robot server.
    */
    using ServerVersion = uint32_t;

    /**
    * Establishes a connection with the robot.
    *
    * @throw NetworkException if the connection is unsuccessful.
    * @throw IncompatibleVersionException if this version of `libperseus` is not supported.
    */
    explicit PerseusRobot();

    /**
    * Move-constructs a new PerseusRobot instance.
    *
    * @param[in] other Other PerseusRobot instance.
    */
    PerseusRobot(PerseusRobot&& other) noexcept;

    /**
    * Move-assigns this PerseusRobot from another PerseusRobot instance.
    *
    * @param[in] other Other PerseusRobot instance.
    *
    * @return PerseusRobot instance.
    */
    PerseusRobot& operator=(PerseusRobot&& other) noexcept;

    /**
    * Closes the connection.
    */
    virtual ~PerseusRobot() noexcept;

    /**
    * Creates a new PerseusRobot instance and returns a shared pointer to it.
    *
    * @return Shared pointer to the newly created PerseusRobot instance.
    */
    static std::shared_ptr<PerseusRobot> Create(); 

    /**
    * Attempts to establish a connection with the hardware device.
    *
    * @return true : Hardware connection succeeded.
    * @return false: Hardware connection failed.
    */
    bool HardwareConnect();

    /**
    * Stops all currently running motions.
    *
    * If a control or motion generator loop is running in another thread, it will be preempted
    * with a perseus::ControlException.
    *
    * @throw CommandException if the Control reports an error.
    * @throw NetworkException if the connection is lost, e.g. after a timeout.
    */
    void Stop();

    void Control(const ControllerMode& controller_mode, std::unique_ptr<RobotCommand> cmd);

    // /**
    // * Starts a control loop for sending joint positions.
    // *
    // * Sets realtime priority for the current thread.
    // * Cannot be executed while another control or motion generator loop is active.
    // *
    // * @param[in] control_callback Callback function providing joint-level torque commands.
    // * See @ref callback-docs "here" for more details.
    // *
    // * @throw ControlException if an error related to torque control or motion generation occurred.
    // * @throw InvalidOperationException if a conflicting operation is already running.
    // * @throw NetworkException if the connection is lost, e.g. after a timeout.
    // * @throw RealtimeException if realtime priority cannot be set for the current thread.
    // * @throw std::invalid_argument if joint-level torque or joint position commands are NaN or
    // * infinity.
    // *
    // * @see PerseusRobot::PerseusRobot to change behavior if realtime priority cannot be set.
    // */
    // void control(std::function<void(std::shared_ptr<PerseusRobot>)> control_callback, 
    //              const std::array<double,JOINT_NUM> &desired_joint, double timeout = 30);

    /**
    * Waits for a robot state update and returns it.
    *
    * Cannot be executed while a control or motion generator loop is running.
    *
    * @return Current daularm-robot state.
    *
    * @throw InvalidOperationException if a conflicting operation is already running.
    * @throw NetworkException if the connection is lost, e.g. after a timeout.
    *
    * @see PerseusRobot::read for a way to repeatedly receive the robot state.
    */
    virtual std::shared_ptr<RobotState> ReadOnce();

    /**
    * Returns the software version reported by the connected robot-server.
    *
    * @return Software version of the connected robot-server.
    */
    ServerVersion getServerVersion() const noexcept;

    /// @cond DO_NOT_DOCUMENT
    PerseusRobot(const PerseusRobot&) = delete;
    PerseusRobot& operator=(const PerseusRobot&) = delete;
    /// @endcond

    class Impl;

protected:
    /**
    * Constructs a new PerseusRobot given a PerseusRobot::Impl. This enables unittests with PerseusRobot::Impl-Mocks.
    *
    * @param robot_impl PerseusRobot::Impl to use
    */
    PerseusRobot(std::shared_ptr<Impl> robot_impl);

private:
    std::shared_ptr<Impl> impl_;
    std::mutex control_mutex_;
};

}  // namespace wisson_SDK 