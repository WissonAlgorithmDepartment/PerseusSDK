/**
 * @file perseus_log.h
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.1
 * @date: 2025-04-26
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief Provides asynchronous logging management using spdlog.
 *
 * LoggerManager supports daily file logging, console logging, and asynchronous logging with thread-safe access.
 * It also provides task-specific loggers and configurable log levels.
 */
#pragma once

#include <memory>
#include <string>
#include <chrono>

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>


namespace wisson_SDK::logging {

/**
 * @class LoggerManager
 * @brief Centralized logging manager for the robot system using spdlog.
 *
 * Features:
 * - Asynchronous logging to file and console
 * - Daily log rotation
 * - Thread-safe initialization using `std::once_flag`
 * - Configurable log levels
 */
class LoggerManager : public std::enable_shared_from_this<LoggerManager>
{
public:
    /**
     * @brief Initializes the main logging system.
     * @param path File path for the main log (default: "../logs/main.log").
     * This will setup asynchronous logging to file and console.
     */
    static void InitLogging(const std::string& path = "../logs/main.log"); 
    
    /**
     * @brief Sets global log level for all loggers.
     * @param level Log level as string ("debug", "info").
     */
    static void SetLogLevel(const std::string& level);

private:
    LoggerManager() = default;

    /**
     * @brief Creates the main logger that combines console and file sinks.
     * @param path Log file path.
     * @return Shared pointer to spdlog logger.
     */
    static std::shared_ptr<spdlog::logger> MainLogger(const std::string& path);

    /**
     * @brief Creates a daily-rotated file logger.
     * @tparam Args Additional constructor arguments.
     * @param name Logger name.
     * @param args Additional arguments forwarded to spdlog sink constructor.
     * @return Shared pointer to spdlog logger.
     */
    template <typename... Args>
    static std::shared_ptr<spdlog::logger> CreateDailyLogger(const std::string& name, Args&&... args);
    
    /**
     * @brief Returns a shared pointer to a console logger (colored output).
     * @return Shared pointer to spdlog logger.
     */
    static std::shared_ptr<spdlog::logger> ConsoleLogger(); 
    
    /**
     * @brief Creates a custom asynchronous logger.
     * @param path Log file path
     * @param name Logger name
     * @param suffix File suffix
     * @return Shared pointer to spdlog async logger
     */
    static std::shared_ptr<spdlog::logger> CustomAsyncLogger(const std::string& path, 
                                                             const std::string& name, 
                                                             const std::string& suffix);

    /**
     * @brief Initializes a global thread pool for asynchronous loggers.
     */
    static void CustomThreadPool();

    /**
     * @brief Creates a directory if it does not exist.
     * @param path Directory path
     * @return true if directory exists or was created successfully
     */
    static bool CreateDir(const std::string& path);

private:
    static std::once_flag pool_flag_; ///< Ensures thread-safe thread-pool initialization
};

}  // namespace wisson_SDK::logging