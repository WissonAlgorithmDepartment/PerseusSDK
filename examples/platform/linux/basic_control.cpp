/**
 * Copyright (c) 2025, WissonRobotics
 * File: basic_control.cpp
 * Author: Yuchen Xia (xiayuchen66@gmail.com)
 * Version 1.0
 * Date: 2025-08-29
 * Brief:
 */

//=== Standard library headers ===//
#include <array>
#include <chrono>        
#include <memory>        
#include <pthread.h>
#include <thread>   
#include <filesystem>

//=== Third-party library headers ===//
#include "perseuslib/perseus_robot.h"
#include "perseuslib/controller/controller.h"
#include "logging/perseus_log.h"


int main(int argc, char** argv) 
{
  namespace ctrl = wisson_SDK::control;

  // Set main thread name
  pthread_setname_np(pthread_self(), "Demo_Ctrl");

  // Log initialization
  wisson_SDK::logging::LoggerManager::InitLogging();
  const std::string example_tag = "Basic-Ctrl";

  /*********************************  PerseusRobot-SDK init begin  *********************************/
  std::filesystem::path config_path = std::filesystem::path(CONFIG_PATH) / "config.yaml"; 
  auto robot = wisson_SDK::PerseusRobot::Create(config_path);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  auto mode = ctrl::ControllerMode::JointPosition();
  std::array<double, 9> desired_joint = {0.4280, 30.0, 40.0, -1.0, 2.0, 30.0, 30.0, 30.0, 5.0};
  double timeout = 5.0;
  auto command = ctrl::MotionCommand::CreateCommand(desired_joint, timeout);
  auto cmd = ctrl::RobotCommand::CreateCommand(command);

  robot->Control(mode, std::move(cmd));
  return 0;
}