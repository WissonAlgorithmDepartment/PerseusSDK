cmake_minimum_required(VERSION 3.11...3.26)

# =======================================================================================
#                                    Utils
# =======================================================================================
include(cmake/utils.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/log.cmake)

set(ENABLE_COLOR_OUTPUT ON)

set(IS_TOP_LEVEL_PROJECT FALSE)
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(IS_TOP_LEVEL_PROJECT TRUE)
  set(PROJECT_TAG "Top")
else()
  set(PROJECT_TAG "Sub")
endif ()

if (IS_TOP_LEVEL_PROJECT)
  #perseus_print_top_separator()
else()
  perseus_print_sub_separator_start()
endif ()



# =======================================================================================
#                           Extract version from version.h
# =======================================================================================
PerseusSDK_extract_version()
 


# =======================================================================================
#                                 Project Info
# =======================================================================================
project (PerseusSDK 
  VERSION ${PERSEUSSDK_VERSION} 
  LANGUAGES CXX
  DESCRIPTION "Wisson Perseus-SDK"
)

# Disable in-source builds to prevent source tree corruption.
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  perseus_error("FATAL: In-source builds are not allowed.
    CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}, CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}
    You should create a separate directory for build files.")
endif ()

perseus_newline()
perseus_print("[${PROJECT_TAG}] Starting Perseus-SDK configuration...")
perseus_print("----------------------------------------------------------------------")
perseus_print("    Building [Wisson-${PROJECT_TAG}] ${PROJECT_NAME} Version: ${PERSEUSSDK_VERSION}")
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
  if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(PLATFORM "Linux ARM64")
    set(PLATFORM_ARM64 ON)
    set(3RD_PARTY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/aarch64")
  elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
    set(PLATFORM "Linux x86_64")
    set(PLATFORM_X86_64 ON)
    set(3RD_PARTY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/x86_64")
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

perseus_print("[${FULL_TAG}] Detected platform: ${PLATFORM}")



# =======================================================================================
#                            System Libraries & Install
# =======================================================================================
if (${PLATFORM_WINDOWS})
  include(InstallRequiredSystemLibraries)
endif ()

include(GNUInstallDirs)



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

set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

add_compile_options(-Wall -Wextra -Wpedantic)

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
# Examples
option(PERSEUSSDK_BUILD_EXAMPLES "Build example code" ON)



# =======================================================================================
#                               Examples & Tests
# =======================================================================================
perseus_newline()
perseus_newline()
if (PERSEUSSDK_BUILD_EXAMPLES)
  perseus_print("[${FULL_TAG}] Building examples")
  add_subdirectory(examples/platform/linux)
else()
  perseus_print("[${FULL_TAG}] Skipping examples")
endif ()



# =======================================================================================
#                         Configure Build Log Summary
# =======================================================================================
perseus_print_fixed_header("${PROJECT_NAME} Configure Summary")
perseus_info("Project                   : " "${PROJECT_NAME}")
perseus_info("Version                   : " "${PROJECT_VERSION}")

perseus_print_fixed_header("${PROJECT_NAME} Build Options Summary")
perseus_info("Build Examples            : " "${PERSEUSSDK_BUILD_EXAMPLES}")
perseus_info("Build Tests               : " "${PERSEUSSDK_BUILD_TESTS}")
perseus_info("Generate install target   : " "${PERSEUSSDK_INSTALL}")
perseus_info("Generate uninstall target : " "${PERSEUSSDK_UNINSTALL}")
perseus_info("Address sanitizer         : " "${PERSEUSSDK_SANITIZE_ADDRESS}")
perseus_print_fixed_header("")

perseus_success("Successfully finished ${PROJECT_NAME} configuration!")
if (IS_TOP_LEVEL_PROJECT)
  #perseus_print_top_separator()
else()
  perseus_print_sub_separator()
endif ()