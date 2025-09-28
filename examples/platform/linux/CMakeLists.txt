cmake_minimum_required(VERSION 3.11...3.26)

# =======================================================================================
#                                    Utils
# =======================================================================================
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

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
set(PLATFORM_ARM64 OFF)
set(PLATFORM_X86_64 OFF)
set(PLATFORM_WINDOWS OFF)
set(3RD_PARTY_PATH "")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_AARCH64=1)
    set(3RD_PARTY_PATH "${CMAKE_SOURCE_DIR}/3rd_party/aarch64")
  elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_X86_64=1)
    set(3RD_PARTY_PATH "${CMAKE_SOURCE_DIR}/3rd_party/x86_64")
  else()
    perseus_error("!!! FATAL: Unsupported Linux architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
elseif (WIN32)
  set(PLATFORM "Windows x86_64")
  set(PLATFORM_WINDOWS ON)
  set(3RD_PARTY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/windows")
else()
  perseus_error("!!! FATAL: Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()



# =======================================================================================
#                          Global configs (safe defaults)
# =======================================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_C_COMPILER)
  set(CMAKE_C_COMPILER gcc)
endif()

if(NOT CMAKE_CXX_COMPILER)
  set(CMAKE_CXX_COMPILER g++)
endif()

# 调试信息
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

# 开启所有警告
add_compile_options(-Wall -Wextra -Wpedantic)

# 默认构建类型
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif ()

# 生成 compile_commands.json (便于 IDE/clangd)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 可视化 verbose makefile（如需更详细编译输出可设为 ON）
option(PERSEUS_VERBOSE_MAKEFILE "Enable verbose makefile output" OFF)
if (PERSEUS_VERBOSE_MAKEFILE)
  set(CMAKE_VERBOSE_MAKEFILE ON)
endif ()

# 位置无关代码（对静态/动态库都安全）
set(CMAKE_POSITION_INDEPENDENT_CODE ON)



# =======================================================================================
#                               Build Options
# =======================================================================================
option(PERSEUSSDK_STACK_TRACE "Enable stack trace support in Perseus SDK" OFF)

option(PERSEUS_BOOST_USE_STATIC_LIBS "Link Boost statically" ON)
option(PERSEUS_BOOST_USE_STATIC_RUNTIME "Use static runtime (MSVC /MT)" OFF)

set(PERSEUS_SPDLOG_ACTIVE_LEVEL "SPDLOG_LEVEL_DEBUG" CACHE STRING "spdlog active level")



# =======================================================================================
#                              Dependencies
# =======================================================================================
if(NOT 3RD_PARTY_PATH)
  perseus_error("[${FULL_TAG}] " "3RD_PARTY_PATH is not set")
endif()

function(check_required_dependency DEPENDENCY_NAME)
  if (${DEPENDENCY_NAME}_FOUND)
    perseus_success("[${FULL_TAG}] Found ${DEPENDENCY_NAME}")
  else()
    perseus_warn("[${FULL_TAG}] " "Could NOT find ${DEPENDENCY_NAME} - some features may be disabled")
  endif ()
endfunction()

function(check_optional_dependency DEPENDENCY_NAME)
  if (${DEPENDENCY_NAME}_FOUND)
    perseus_print("[${FULL_TAG}] Found optional ${DEPENDENCY_NAME}")
  else()
    perseus_warn("[${FULL_TAG}] " "Optional ${DEPENDENCY_NAME} not found - skipping")
  endif ()
endfunction()

perseus_print("[${FULL_TAG}] Checking for required dependencies...")

# jsoncpp
list(APPEND CMAKE_PREFIX_PATH "${3RD_PARTY_PATH}/jsoncpp")
perseus_print("[${FULL_TAG}] Added JSONCPP to CMAKE_PREFIX_PATH")
find_package(jsoncpp CONFIG REQUIRED)
check_required_dependency(jsoncpp)

if (TARGET JsonCpp::JsonCpp)  
  set(jsoncpp_LIBRARIES JsonCpp::JsonCpp)
  
  get_target_property(jsoncpp_INCLUDE_DIR JsonCpp::JsonCpp INTERFACE_INCLUDE_DIRECTORIES)
  
  if(NOT jsoncpp_INCLUDE_DIR)
    get_target_property(jsoncpp_INCLUDE_DIR JsonCpp::JsonCpp INCLUDE_DIRECTORIES)
  endif()
  
  if(NOT jsoncpp_INCLUDE_DIR)
    set(jsoncpp_INCLUDE_DIR "${3RD_PARTY_PATH}/jsoncpp/include")
  endif()
  
elseif (DEFINED JSONCPP_INCLUDE_DIRS)
  set(jsoncpp_INCLUDE_DIR ${JSONCPP_INCLUDE_DIRS})
  set(jsoncpp_LIBRARIES ${JSONCPP_LIBRARIES})
else()
  perseus_error("[${FULL_TAG}] Could not determine jsoncpp include directory")
endif()

# yaml-cpp
list(APPEND CMAKE_PREFIX_PATH "${3RD_PARTY_PATH}/yaml-cpp")
perseus_print("[${FULL_TAG}] Added YAML-CPP to CMAKE_PREFIX_PATH")
find_package(yaml-cpp REQUIRED)
check_required_dependency(yaml-cpp)
get_target_property(yamlcpp_INCLUDE_DIR yaml-cpp::yaml-cpp INTERFACE_INCLUDE_DIRECTORIES)

# # Eigen3（优先 CONFIG，兼容 NO_MODULE）
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# check_required_dependency(Eigen3)

# -------------------------------------------------------------
# Boost (Cross-compile aarch64) find module for Perseus
# Only search system library, no test compilation
# -------------------------------------------------------------
set(BOOST_ROOT "${3RD_PARTY_PATH}/boost")
list(APPEND CMAKE_PREFIX_PATH "${BOOST_ROOT}")
perseus_print("[${FULL_TAG}] Added Boost to CMAKE_PREFIX_PATH")

# Disable automatic testing and system paths
set(Boost_NO_SYSTEM_PATHS ON)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_NO_BOOST_CMAKE ON)

# Explicitly specify include and library directories
set(Boost_INCLUDE_DIR "${BOOST_ROOT}/include")
set(Boost_LIBRARY_DIR "${BOOST_ROOT}/lib")

# Boost.System static library path
set(Boost_SYSTEM_LIBRARY "${Boost_LIBRARY_DIR}/libboost_system.a")

# Check existence
if(NOT EXISTS "${Boost_INCLUDE_DIR}/boost/system/system_error.hpp")
  perseus_error("[${FULL_TAG}] " "Boost include directory not found: ${Boost_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${Boost_SYSTEM_LIBRARY}")
  perseus_error("[${FULL_TAG}] " "Boost.System library not found: ${Boost_SYSTEM_LIBRARY}")
endif()

# Create IMPORTED target for Boost::system
add_library(Boost::system STATIC IMPORTED GLOBAL)
set_target_properties(Boost::system PROPERTIES
  IMPORTED_LOCATION "${Boost_SYSTEM_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIR}"
)

# Mark found
set(Boost_FOUND TRUE)
set(Boost_INCLUDE_DIRS "${Boost_INCLUDE_DIR}")
set(Boost_LIBRARIES "${Boost_SYSTEM_LIBRARY}")
set(Boost_SYSTEM_LIBRARY "${Boost_SYSTEM_LIBRARY}")

perseus_print("[${FULL_TAG}] Found Boost.System")

# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
check_required_dependency(Threads)

# spdlog
list(APPEND CMAKE_PREFIX_PATH "${3RD_PARTY_PATH}/spdlog")
perseus_print("[${FULL_TAG}] Added SPDLOGP to CMAKE_PREFIX_PATH")

set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "Use spdlog internal fmt library" FORCE)
perseus_print("[${FULL_TAG}] SPDLOG_FMT_EXTERNAL = ${SPDLOG_FMT_EXTERNAL}")

find_package(spdlog REQUIRED)
check_required_dependency(spdlog)

set(ALL_REQUIRED_DEPS_FOUND TRUE)
if (NOT Boost_FOUND OR NOT Threads_FOUND OR NOT spdlog_FOUND OR NOT yaml-cpp_FOUND OR NOT jsoncpp_FOUND)
  set(ALL_REQUIRED_DEPS_FOUND FALSE)
endif ()

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
  path_control
)

set(EXAMPLE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DEXAMPLE_PATH_STR="${EXAMPLE_PATH}")

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_include_directories(${example} PUBLIC
    ${CMAKE_SOURCE_DIR}
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

# Enable stack trace support if requested
if (PERSEUSSDK_STACK_TRACE)
  perseus_print("[${FULL_TAG}] Building with stack trace support")

  # Link required libraries
  target_link_libraries(stack_trace PRIVATE dw elf)

  # Add compile definition for source code
  target_compile_definitions(stack_trace PRIVATE PERSEUSSDK_ENABLE_STACK_TRACE)
endif ()

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
perseus_info("Project                    : " "${PROJECT_NAME}")
perseus_info("Version                    : " "${PROJECT_VERSION}")
perseus_info("Build Type                 : " "${CMAKE_BUILD_TYPE}")
perseus_info("C++ Compiler               : " "${CMAKE_CXX_COMPILER} (ID: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION})")
perseus_info("CXX Standard               : " "${CMAKE_CXX_STANDARD} (required=${CMAKE_CXX_STANDARD_REQUIRED}, ext=${CMAKE_CXX_EXTENSIONS})")
perseus_info("Generator                  : " "${CMAKE_GENERATOR}")
perseus_info("System                     : " "${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
perseus_info("Install Prefix             : " "${CMAKE_INSTALL_PREFIX}")
perseus_info("Config Path                : " "${CONFIG_PATH}")

perseus_print_fixed_header("${PROJECT_NAME} Build Options Summary")
perseus_info("Enable stack trace support : " "${PERSEUSSDK_STACK_TRACE}")

perseus_print_fixed_header("${PROJECT_NAME} Dependencies Summary")
perseus_info("jsoncpp Include            : " "${jsoncpp_INCLUDE_DIR}")
perseus_info("yaml-cpp Include           : " "${yamlcpp_INCLUDE_DIR}")
perseus_info("Boost Static Libs          : " "${Boost_USE_STATIC_LIBS}")
perseus_info("Boost Libs                 : " "${Boost_LIBRARIES}")
perseus_info("Boost Include              : " "${Boost_INCLUDE_DIRS}")
perseus_info("Threads                    : " "${CMAKE_THREAD_LIBS_INIT}")
perseus_info("spdlog DIR                 : " "${spdlog_DIR}")
perseus_info("spdlog Level               : " "${PERSEUS_SPDLOG_ACTIVE_LEVEL}")
perseus_print_fixed_header("")

perseus_success("Successfully finished ${PROJECT_NAME} configuration!")
if(IS_TOP_LEVEL_PROJECT)
  perseus_print_top_separator()
else()
  perseus_print_sub_separator()
endif()