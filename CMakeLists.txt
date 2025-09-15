cmake_minimum_required(VERSION 3.11...3.26)

# =======================================================================================
#                                    Utils
# =======================================================================================
include(cmake/utils.cmake)

set(ENABLE_COLOR_OUTPUT ON)
include(${CMAKE_SOURCE_DIR}/cmake/log.cmake)

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
if (CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
  set(LIBRARY_PATH perseus_lib/lib/libPERSEUSSDK.a)
else ()
  perseus_error("FATAL: Unsupported platform detected!\n"
                "Only Linux-aarch64 is currently supported.\n"
                "Please check your system architecture.")
endif ()



# =======================================================================================
#                            System Libraries & Install
# =======================================================================================
if (${WINDOWS})
  include(InstallRequiredSystemLibraries)
endif ()

include(GNUInstallDirs)



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