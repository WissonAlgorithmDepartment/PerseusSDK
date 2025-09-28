/**
 * Copyright (c) 2025, WissonRobotics
 * File: stack_trace.cpp
 * Author: Yuchen Xia (xiayuchen66@gmail.com)
 * Version 1.0
 * Date: 2025-09-15
 * Brief:
 */

//=== Standard library headers ===//     
#include <csignal>       
#include <vector>
#include <array>
#include <chrono>        
#include <memory>        
#include <pthread.h>
#include <thread> 
#include <filesystem>

//=== Third-party library headers ===//
#include "perseuslib/perseus_robot.h"
#include "logging/perseus_log.h"

#ifdef PERSEUSSDK_ENABLE_STACK_TRACE
  #define BACKWARD_HAS_DW 1  // Enable Backward-cpp with libdw support
  #include "logging/backward.hpp"

  static std::vector<int> build_signals()
  {
    auto v = backward::SignalHandling::make_default_signals();
    v.push_back(SIGTERM);
    v.push_back(SIGINT);
    return v;                       
  }
#endif


int main(int argc, char** argv) 
{
  std::string thread_name = "Demo_Stack";
  pthread_setname_np(pthread_self(), thread_name.c_str());
  wisson_SDK::logging::LoggerManager::InitLogging();

#ifdef PERSEUSSDK_ENABLE_STACK_TRACE
  // Install signal handlers for stack trace
  static backward::SignalHandling sh(build_signals());
#endif

  /*********************************  PerseusRobot-SDK init begin  *********************************/
  std::filesystem::path config_path = std::filesystem::path(CONFIG_PATH) / "config.yaml"; 
  auto robot = wisson_SDK::PerseusRobot::Create(config_path);

  while(true) {
    SPDLOG_INFO("[{}] Waiting stop signal...", thread_name);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  return 0;
}