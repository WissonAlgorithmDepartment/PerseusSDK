/**
 * @file perseus_log.h
 * 
 * @copyright (c) 2025, WissonRobotics
 * 
 * @version 1.1
 * @date: 2025-04-26
 * @author: Yuchen Xia (xiayuchen66@gmail.com)
 * 
 * @brief 
 */
#pragma once

#include <memory>
#include <chrono>

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace wisson_SDK::logging {

enum class TaskRecordType {
    Start = 0,
    Step  = 1,
    End   = 2
};

// 日志管理器
class LoggerManager : public std::enable_shared_from_this<LoggerManager>
{
public:
    static void InitLogging(); 
    
    /* Custom Logger Style */
    static void SetLogLevel(const std::string& level);

private:
    // 主业务日志器
    static std::shared_ptr<spdlog::logger> MainLogger();

    // 带日期的文件日志器模板
    template <typename... Args>
    static std::shared_ptr<spdlog::logger> CreateDailyLogger(const std::string& name, Args&&... args);
    
    static std::shared_ptr<spdlog::logger> ConsoleLogger();                            // 控制台日志器           
    static std::shared_ptr<spdlog::logger> CustomAsyncLogger(const std::string& path, 
                                                             const std::string& name, 
                                                             const std::string& suffix);

    /* Custem Logger Style */
    static void CustomThreadPool();
    static bool CreateDir(const std::string& path);
    // void thread_monitor();

    static std::once_flag pool_flag_;

    std::shared_ptr<spdlog::logger> task_logger_;
};

}  // namespace wisson_SDK::logging