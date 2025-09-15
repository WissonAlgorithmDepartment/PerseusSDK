/**
 * Copyright (c) 2025, WissonRobotics
 * File: basic_control.cpp
 * Author: Yuchen Xia (xiayuchen66@gmail.com)
 * Version 1.0
 * Date: 2025-08-29
 * Brief:
 */

//=== 标准库头文件 ===//
#include <cstdio>        // 替换<stdio.h>
#include <cstdlib>       // 替换<stdlib.h>
#include <csignal>       // 替换<signal.h>
#include <vector>
#include <chrono>        // 时间相关操作
#include <thread>        // 线程支持
#include <fstream>       // 文件操作
#include <iomanip>       // 格式化输出
#include <sstream>       // 字符串流
#include <memory>        // 智能指针
#include <pthread.h>

//=== 第三方库头文件 ===//
#define BACKWARD_HAS_DW 1  // Backward-cpp 堆栈跟踪
#include "perseuslib/version.h"
#include "perseuslib/perseus_robot.h"
#include "perseuslib/controller/controller.h"

#include "logging/backward.hpp"
#include "logging/perseus_log.h"

#ifndef USING_SIMULATION
#define USING_SIMULATION 0
#endif

static std::vector<int> build_signals()
{
  auto v = backward::SignalHandling::make_default_signals();
  v.push_back(SIGTERM);
  v.push_back(SIGINT);
  return v;                       
}

int main(int argc, char** argv) 
{
  // 设置主线程名字
  pthread_setname_np(pthread_self(), "perseuslib");

  // log 初始化; 堆栈跟踪
  wisson_SDK::logging::LoggerManager::InitLogging();
  static backward::SignalHandling sh(build_signals());
    
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=-=- New Session Started -=-=-=-=-=-=-=-=-=-=-=-=");
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=- Perseus-Library : V{} --=-=-=-=-=-=-=-=-=-=", PERSEUSLIB_VERSION);
  SPDLOG_INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
  SPDLOG_INFO("                                                                 ");

  const std::string example_tag = "Basic_Control";

  /*********************************  PerseusRobot-SDK init begin  *********************************/
  auto robot = wisson_SDK::PerseusRobot::Create();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::array<double, 9> desired_joint = {0.4280, 30.0, 40.0, -1.0, 2.0, 30.0, 30.0, 30.0, 5.0};
  wisson_SDK::ControllerMode mode = wisson_SDK::ControllerMode::JointPosition();
  auto cmd_ptr = wisson_SDK::RobotCommand::JointPosition(desired_joint, 10.0);
  robot->Control(mode, std::move(cmd_ptr));

  auto state = robot->ReadOnce(); 
  SPDLOG_INFO("[Basic-Ctrl] Current pressure: {}", fmt::join(state->pressure, ", "));

  return 0;
}