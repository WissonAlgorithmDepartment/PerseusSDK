/**
 * Copyright (c) 2025, WissonRobotics
 * File: path_control.cpp
 * Author: Yuchen Xia (xiayuchen66@gmail.com)
 * Version 1.0
 * Date: 2025-09-15
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
  // Set main thread name
  pthread_setname_np(pthread_self(), "Demo_Path_Ctrl");

  // Log initialization
  wisson_SDK::logging::LoggerManager::InitLogging();
  const std::string example_tag = "Path-Ctrl";

  /*********************************  PerseusRobot-SDK init begin  *********************************/
  std::filesystem::path config_path = std::filesystem::path(CONFIG_PATH) / "config.yaml";
  auto robot = wisson_SDK::PerseusRobot::Create(config_path);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  namespace ctrl = wisson_SDK::control;
  auto mode = ctrl::ControllerMode::JointPosition();
  std::array<double, 9> joint1 = {0.4280, 30.0, 40.0, -1.0, 2.0, 30.0, 30.0, 30.0, 5.0};
  std::array<double, 9> joint2 = {0.4280, 30.0, 40.0, -1.0, 2.0, 30.0, 30.0, 0.0, 35.0};
  double timeout1 = 5.0;
  double timeout2 = 5.0;
  double total_timeout = 30.0;
  std::vector<ctrl::MotionCommand> commands = {
    ctrl::MotionCommand::CreateCommand(joint1, timeout1),
    ctrl::MotionCommand::CreateCommand(joint2, timeout2)
  };
  auto cmd1 = ctrl::RobotCommand::CreateCommands(commands, total_timeout);
  robot->Control(mode, std::move(cmd1));

  auto ee_mode = ctrl::ControllerMode::TaskCommand();
  auto ee_cmd1 = ctrl::EndEffectorAction::Open;
  auto ee_cmd2 = ctrl::EndEffectorAction::ForceClose;
  timeout1 = 5.0;
  timeout2 = 5.0;
  total_timeout = 30.0;
  auto cmd2 = ctrl::RobotCommand::CreateCommands(
    std::vector<ctrl::EndEffectorCommand>{
      {.ee_action = ee_cmd1, .timeout = timeout1}, 
      {.ee_action = ee_cmd2, .timeout = timeout2}
    },
    total_timeout
  );
  robot->Control(ee_mode, std::move(cmd2));

  return 0;
}