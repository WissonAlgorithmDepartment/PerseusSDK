cmake_minimum_required(VERSION 3.11...3.26)
cmake_policy(SET CMP0144 NEW)  

# =======================================================================================
#                                    Utils
# =======================================================================================
# 可选颜色输出
set(ENABLE_COLOR_OUTPUT ON)
include(${CMAKE_SOURCE_DIR}/cmake/log.cmake)

set(IS_TOP_LEVEL_PROJECT FALSE)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(IS_TOP_LEVEL_PROJECT TRUE)
  set(PROJECT_TAG "Top")
else()
  set(PROJECT_TAG "Sub")
endif()

if(IS_TOP_LEVEL_PROJECT)
  perseus_print_top_separator()
else()
  perseus_print_sub_separator_start() 
endif()



# =======================================================================================
#                                 Project Info
# =======================================================================================
project (perseus_sdk_demo
  LANGUAGES CXX
  DESCRIPTION "Perseus Library Examples"
)

perseus_print("[${PROJECT_TAG}] Starting ${PROJECT_NAME} configuration...")
perseus_print("----------------------------------------------------------------------")
perseus_print("    Building [Wisson-${PROJECT_TAG}] ${PROJECT_NAME}")
perseus_print("----------------------------------------------------------------------")

set(FULL_TAG "${PROJECT_TAG}|${PROJECT_NAME}")



# =======================================================================================
#                               Check System Platform
# =======================================================================================
if (CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
  set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
  add_definitions(-DPLATFORM_ARCH_AARCH64=1)
else ()
  perseus_error("FATAL: Unsupported platform detected!\n"
                "Only Linux-aarch64 is currently supported.\n"
                "Please check your system architecture.")
endif ()



# =======================================================================================
#                          Global configs (safe defaults)
# =======================================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CPP_COMPILER g++)
set (CMAKE_CXX_FLAGS "-lstdc++fs -std=c++2a")
add_definitions("-Wall -g")
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PERSEUS_VERBOSE_MAKEFILE "Enable verbose makefile output" OFF)
if (PERSEUS_VERBOSE_MAKEFILE)
  set(CMAKE_VERBOSE_MAKEFILE ON)
endif ()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)



# =======================================================================================
#                               Build Options
# =======================================================================================
option(PERSEUS_BOOST_USE_STATIC_LIBS "Link Boost statically" ON)
option(PERSEUS_BOOST_USE_STATIC_RUNTIME "Use static runtime (MSVC /MT)" OFF)

set(PERSEUS_SPDLOG_ACTIVE_LEVEL "SPDLOG_LEVEL_DEBUG" CACHE STRING "spdlog active level")



# =======================================================================================
#                              Dependencies
# =======================================================================================
# 定义依赖项检查函数
function(check_required_dependency DEPENDENCY_NAME)
  if (${DEPENDENCY_NAME}_FOUND)
    perseus_success("[${FULL_TAG}] Found ${DEPENDENCY_NAME}")
  else()
    perseus_warn("[${FULL_TAG}] Could NOT find ${DEPENDENCY_NAME} - some features may be disabled")
  endif ()
endfunction()

function(check_optional_dependency DEPENDENCY_NAME)
  if (${DEPENDENCY_NAME}_FOUND)
    perseus_print("[${FULL_TAG}] Found optional ${DEPENDENCY_NAME}")
  else()
    perseus_warn("[${FULL_TAG}] Optional ${DEPENDENCY_NAME} not found - skipping")
  endif ()
endfunction()

# 开始依赖项检查
perseus_print("[${FULL_TAG}] Checking for required dependencies...")

# PkgConfig
find_package(PkgConfig REQUIRED)
check_required_dependency(PkgConfig)

# jsoncpp
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/3rd_party/jsoncpp/install")
find_package(jsoncpp CONFIG REQUIRED)
check_required_dependency(jsoncpp)

if (TARGET JsonCpp::JsonCpp)  
  # 方法1：直接使用目标属性
  set(jsoncpp_LIBRARIES JsonCpp::JsonCpp)
  
  # 方法2：尝试获取包含目录的不同方式
  get_target_property(jsoncpp_INCLUDE_DIR JsonCpp::JsonCpp INTERFACE_INCLUDE_DIRECTORIES)
  
  # 如果上面没获取到，尝试其他属性名
  if(NOT jsoncpp_INCLUDE_DIR)
    get_target_property(jsoncpp_INCLUDE_DIR JsonCpp::JsonCpp INCLUDE_DIRECTORIES)
  endif()
  
  # 如果仍然没获取到，手动设置路径
  if(NOT jsoncpp_INCLUDE_DIR)
    set(jsoncpp_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3rd_party/jsoncpp/install/include")
  endif()
  
elseif (DEFINED JSONCPP_INCLUDE_DIRS)
  set(jsoncpp_INCLUDE_DIR ${JSONCPP_INCLUDE_DIRS})
  set(jsoncpp_LIBRARIES ${JSONCPP_LIBRARIES})
else()
  perseus_error("[${FULL_TAG}] Could not determine jsoncpp include directory")
endif()

# yaml-cpp
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/3rd_party/yaml-cpp/install")
find_package(yaml-cpp REQUIRED)
check_required_dependency(yaml-cpp)
get_target_property(yamlcpp_INCLUDE_DIR yaml-cpp::yaml-cpp INTERFACE_INCLUDE_DIRECTORIES)

# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
check_required_dependency(Threads)

# spdlog
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/3rd_party/spdlog-1.12.0/install")
find_package(spdlog REQUIRED)
check_required_dependency(spdlog)

# Boost.System
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_NO_BOOST_CMAKE ON)

set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/3rd_party/boost")
set(BOOST_INCLUDEDIR "${BOOST_ROOT}/include")
set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib")
set(Boost_NO_SYSTEM_PATHS ON)

set(BOOST_COMPONENTS
  system
)

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
check_required_dependency(Boost)

# 检查所有必需依赖是否满足
set(ALL_REQUIRED_DEPS_FOUND TRUE)
if (NOT Boost_FOUND OR NOT Threads_FOUND OR NOT spdlog_FOUND)
  set(ALL_REQUIRED_DEPS_FOUND FALSE)
endif ()

# 输出总结信息
if (ALL_REQUIRED_DEPS_FOUND)
  perseus_print("[${FULL_TAG}] All required dependencies are satisfied!")
else()
  perseus_error("[${FULL_TAG}] Some required dependencies are missing. Please install them and reconfigure.")
endif ()



# =======================================================================================
#                                    Target
# =======================================================================================
link_directories(${CMAKE_SOURCE_DIR}/perseus_lib/lib/${TOOLCHAIN_NAME})
link_libraries(${CMAKE_SOURCE_DIR}/perseus_lib/lib/${TOOLCHAIN_NAME}/libperseuslib.a -lstdc++)

set(CONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DCONFIG_PATH=\"${CONFIG_PATH}\")

set(EXAMPLES
  basic_control
  stack_trace
  logging_example
)

set(EXAMPLE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DEXAMPLE_PATH_STR="${EXAMPLE_PATH}")

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_include_directories(${example} PUBLIC
    ${CMAKE_SOURCE_DIR}/perseus_lib/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${example} PRIVATE 
    perseuslib 
    Threads::Threads
    spdlog::spdlog
    JsonCpp::JsonCpp
    yaml-cpp::yaml-cpp
  )
  
  target_compile_definitions(${example} PRIVATE SPDLOG_ACTIVE_LEVEL=${PERSEUS_SPDLOG_ACTIVE_LEVEL})
endforeach()

target_link_libraries(stack_trace PRIVATE dw elf)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)



# =======================================================================================
#                         Configure Build Log Summary
# =======================================================================================
perseus_newline()
perseus_print("[${FULL_TAG}] Configuration Log Summary")
perseus_print_fixed_header("${PROJECT_NAME} Configure Summary")
perseus_info("Project                   : " "${PROJECT_NAME}")
perseus_info("Version                   : " "${PROJECT_VERSION}")
perseus_info("Build Type                : " "${CMAKE_BUILD_TYPE}")
perseus_info("C++ Compiler              : " "${CMAKE_CXX_COMPILER} (ID: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION})")
perseus_info("CXX Standard              : " "${CMAKE_CXX_STANDARD} (required=${CMAKE_CXX_STANDARD_REQUIRED}, ext=${CMAKE_CXX_EXTENSIONS})")
perseus_info("Generator                 : " "${CMAKE_GENERATOR}")
perseus_info("System                    : " "${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
perseus_info("Install Prefix            : " "${CMAKE_INSTALL_PREFIX}")
perseus_info("Config Path               : " "${CONFIG_PATH}")

perseus_print_fixed_header("${PROJECT_NAME} Dependencies Summary")
perseus_info("jsoncpp Include           : " "${jsoncpp_INCLUDE_DIR}")
perseus_info("yaml-cpp Include          : " "${yamlcpp_INCLUDE_DIR}")
perseus_info("Boost Static Libs         : " "${Boost_USE_STATIC_LIBS}")
perseus_info("Boost Libs                : " "${Boost_LIBRARIES}")
perseus_info("Boost Include             : " "${Boost_INCLUDE_DIRS}")
perseus_info("Threads                   : " "${CMAKE_THREAD_LIBS_INIT}")
perseus_info("spdlog DIR                : " "${spdlog_DIR}")
perseus_info("spdlog Level              : " "${PERSEUS_SPDLOG_ACTIVE_LEVEL}")
perseus_print_fixed_header("")

perseus_success("Successfully finished ${PROJECT_NAME} configuration!")
if(IS_TOP_LEVEL_PROJECT)
  perseus_print_top_separator()
else()
  perseus_print_sub_separator()
endif()