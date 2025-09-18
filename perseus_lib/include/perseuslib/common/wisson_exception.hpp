/**
 * @file wisson_exception.hpp
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.0
 * @date: 2025-08-28
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Defines exception types used throughout libperseus for error handling.
 */
#pragma once

#include <stdexcept>
#include <string>


namespace wisson_SDK {

/**
 * @brief Base class for all exceptions used by libperseus.
 *
 * This serves as a common parent for all exception types in the library,
 * allowing clients to catch `Exception` when they want to handle all
 * Perseus-specific errors uniformly.
 */
struct Exception : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

/**
 * @brief Thrown when object construction fails.
 *
 * Typical cases include invalid arguments, failure during resource allocation,
 * or errors encountered while initializing a robot-related component.
 */
struct ConstructorException : public Exception {
  using Exception::Exception;
};

/**
 * @brief Thrown when network-related operations fail.
 *
 * This includes:
 * - Failure to connect to the robot.
 * - Communication timeouts.
 * - Connection loss during active communication.
 */
struct NetworkException : public Exception {
  using Exception::Exception;
};

/**
 * @brief Thrown when an invalid or unexpected protocol message is received.
 *
 * Examples:
 * - The robot sends a malformed message.
 * - A message does not match the expected protocol schema.
 * - Critical fields are missing in the response.
 */
struct ProtocolException : public Exception {
  using Exception::Exception;
};

/**
 * @brief Thrown when control operations on the robot fail.
 *
 * This typically indicates:
 * - Failure to enter or exit a control mode.
 * - Errors during trajectory execution.
 * - Safety violations preventing the requested action.
 */
struct ControlException : public Exception {
  using Exception::Exception;
};

/**
 * @brief Thrown when a command execution fails.
 *
 * Common scenarios include:
 * - Invalid command parameters.
 * - Execution rejected by the robot.
 * - Failure reported by the robot firmware.
 */
struct CommandException : public Exception {
  using Exception::Exception;
};

/**
 * @brief Thrown when an operation cannot be performed in the current state.
 *
 * This usually occurs if:
 * - A requested operation is logically invalid.
 * - The current robot state does not allow the action (e.g., trying to move while in emergency stop).
 * - The API usage violates expected state transitions.
 */
struct InvalidOperationException : public Exception {
  using Exception::Exception;
};

}  // namespace wisson_SDK 