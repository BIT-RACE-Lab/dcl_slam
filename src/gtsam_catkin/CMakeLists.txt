cmake_minimum_required(VERSION 2.8.3)
project(gtsam_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()

include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

# GTSAM is vendored with this workspace to keep builds independent of external
# network access. The directory must contain commit
# 687ae3d2511b9c296af08ec2f2e717b0627a8d68, including its CMakeLists.txt.
set(GTSAM_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/gtsam"
    CACHE PATH "Path to the local GTSAM source tree")

if(NOT EXISTS "${GTSAM_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "GTSAM source was not found at: ${GTSAM_SOURCE_DIR}\n"
    "Populate src/gtsam_catkin/third_party/gtsam with commit "
    "687ae3d2511b9c296af08ec2f2e717b0627a8d68, then rebuild.\n"
    "To use another local path, configure: "
    "catkin config --cmake-args -DGTSAM_SOURCE_DIR=/absolute/path/to/gtsam")
endif()

ExternalProject_Add(gtsam_src
  SOURCE_DIR "${GTSAM_SOURCE_DIR}"
  DOWNLOAD_COMMAND ""
  UPDATE_COMMAND ""
  # The vendored GTSAM source already contains use_catkinized_metis.patch and
  # fix_warnings.patch. Do not apply them again: patch would wait for input
  # when a package build is cleaned and restarted.
  CONFIGURE_COMMAND ${CMAKE_COMMAND}
  -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
  -DGTSAM_USE_SYSTEM_EIGEN=ON
  -DCMAKE_BUILD_TYPE=Release
  "${GTSAM_SOURCE_DIR}"
  # Inherit catkin_tools GNU Make jobserver tokens (configured via `catkin config -j`).
  # Do not force a private -j value here, which would oversubscribe or underuse CPUs.
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND make install
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/CppUnitLite
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/gtsam
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/gtsam_unstable
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/wrap
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libCppUnitLite*")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libgtsam*")

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES CppUnitLite gtsam gtsam_unstable metis tbb)
