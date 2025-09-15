/**
 * Copyright (c) 2025, WissonRobotics
 * File: logging_example.cpp
 * Author: Yuchen Xia (xiayuchen66@gmail.com)
 * Version 1.0
 * Date: 2025-09-15
 * Brief:
 */

//=== Standard library headers ===//
#include <chrono>        
#include <memory>        
#include <pthread.h>
#include <thread>   
#include <filesystem>

//=== Third-party library headers ===//
#include "perseuslib/perseus_robot.h"
#include "logging/perseus_log.h"

#include "version.h"


int main(int argc, char** argv) 
{
  // Set main thread name
  pthread_setname_np(pthread_self(), "Demo_Logging");

  // Log initialization
  wisson_SDK::logging::LoggerManager::InitLogging();
    
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=-=- New Session Started -=-=-=-=-=-=-=-=-=-=-=-=");
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=- Perseus-SDK : V{} --=-=-=-=-=-=-=-=-=-=", PERSEUSSDK_VERSION);
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
  SPDLOG_INFO("                                                                 ");
  
  const std::string example_tag = "Demo-Logging";
  SPDLOG_WARN("[{}] This is a warning message.", example_tag);
  SPDLOG_ERROR("[{}] This is a error message.", example_tag);

  /*********************************  PerseusRobot-SDK init begin  *********************************/
  std::filesystem::path config_path = std::filesystem::path(CONFIG_PATH) / "config.yaml"; 
  auto robot = wisson_SDK::PerseusRobot::Create(config_path);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  auto state = robot->ReadOnce(); 
  SPDLOG_INFO("[{}] Current pressure: {}", example_tag, fmt::join(state->pressure, ", "));

  return 0;
}