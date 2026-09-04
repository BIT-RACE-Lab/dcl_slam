cmake_minimum_required(VERSION 2.8.3)
project(gflags_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

set(GFLAGS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/gflags"
    CACHE PATH "Path to the local gflags source tree")

if(NOT EXISTS "${GFLAGS_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "gflags source was not found at: ${GFLAGS_SOURCE_DIR}\n"
    "Populate src/gflags_catkin/third_party/gflags, then rebuild.")
endif()

ExternalProject_Add(
  gflags_src
  SOURCE_DIR "${GFLAGS_SOURCE_DIR}"
  DOWNLOAD_COMMAND ""
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
    -DBUILD_SHARED_LIBS:BOOL=true
    -DGFLAGS_NAMESPACE:STRING=google
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DCMAKE_TOOLCHAIN_FILE:STRING=${CMAKE_TOOLCHAIN_FILE}
  # Share catkin_tools' GNU Make jobserver instead of forcing make -j 8.
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/gflags
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libgflags*")
install(FILES ${CATKIN_DEVEL_PREFIX}/bin/gflags_completions.sh
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES gflags)
