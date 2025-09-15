/**
 * @file wisson_exception.h
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.0
 * @date: 2025-08-28
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Contains exception definitions.
 */
#pragma once

#include <stdexcept>
#include <string>


namespace wisson_SDK {

/**
 * Base class for all exceptions used by `libperseus`.
 */
struct Exception : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

/**
 * NetworkException is thrown if a connection to the robot cannot be established, or when a timeout
 * occurs.
 */
struct NetworkException : public Exception {
  using Exception::Exception;
};

/**
 * ProtocolException is thrown if the robot returns an incorrect message.
 */
struct ProtocolException : public Exception {
  using Exception::Exception;
};

struct ControlException : public Exception {
  using Exception::Exception;
};

/**
 * CommandException is thrown if an error occurs during command execution.
 */
struct CommandException : public Exception {
  using Exception::Exception;
};

/**
 * InvalidOperationException is thrown if an operation cannot be performed.
 */
struct InvalidOperationException : public Exception {
  using Exception::Exception;
};

}  // namespace wisson_SDK 