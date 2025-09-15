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
#define BACKWARD_HAS_DW 1  // Backward-cpp stack trace
#include "perseuslib/perseus_robot.h"
#include "logging/backward.hpp"
#include "logging/perseus_log.h"


static std::vector<int> build_signals()
{
  auto v = backward::SignalHandling::make_default_signals();
  v.push_back(SIGTERM);
  v.push_back(SIGINT);
  return v;                       
}

int main(int argc, char** argv) 
{
  // Set main thread name
  pthread_setname_np(pthread_self(), "Demo_Stack");

  // Log initialization; stack trace
  wisson_SDK::logging::LoggerManager::InitLogging();
  static backward::SignalHandling sh(build_signals());

  /*********************************  PerseusRobot-SDK init begin  *********************************/
  std::filesystem::path config_path = std::filesystem::path(CONFIG_PATH) / "config.yaml"; 
  auto robot = wisson_SDK::PerseusRobot::Create(config_path);

  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}