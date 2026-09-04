cmake_minimum_required(VERSION 2.8.3)
project(glog_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

cmake_policy(SET CMP0053 OLD)
include(cmake/glog-catkin-extras.cmake.in)

# We can't check version numbers easily as the GLOG script doesn't export them.

# If set to OFF, will compile the given version of Glog.
# If set to ON, will use the system version of Glog.
# If set to AUTO, will try to use the syste version if there is one, and build otherwise.
set(USE_SYSTEM_GLOG "AUTO" CACHE INTERNAL "Whether to use the system version of Glog.")

# Check if the version is suitable, otherwise force downloading the newer one.
if(USE_SYSTEM_GLOG STREQUAL "AUTO" OR USE_SYSTEM_GLOG STREQUAL "ON")
  find_package(Glog QUIET)
  if(GLOG_FOUND)
    set(USE_SYSTEM_GLOG "ON")
    message(STATUS "Suitable Glog version found.")
  else()
    set(USE_SYSTEM_GLOG "OFF")
    message(STATUS "No suitable Glog version found.")
  endif()
endif()


if(USE_SYSTEM_GLOG STREQUAL "OFF")
  message(STATUS "Installing Glog from source!")

  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

  set(VERSION 0.3.5)
  set(GLOG_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glog"
      CACHE PATH "Path to the local glog source tree")

  if(NOT EXISTS "${GLOG_SOURCE_DIR}/configure")
    message(FATAL_ERROR
      "glog source was not found at: ${GLOG_SOURCE_DIR}\n"
      "Populate src/glog_catkin/third_party/glog, then rebuild.")
  endif()

  ExternalProject_Add(glog_src
    SOURCE_DIR "${GLOG_SOURCE_DIR}"
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    # The vendored source already contains fix-unused-typedef-warning.patch.
    CONFIGURE_COMMAND ${GLOG_SOURCE_DIR}/configure --with-pic
      --with-gflags=${gflags_catkin_PREFIX}
      --prefix=${CATKIN_DEVEL_PREFIX}
    BUILD_COMMAND $(MAKE)
    INSTALL_COMMAND $(MAKE) install
  )

  # A fresh Git checkout gives the source files new timestamps. In glog 0.3.5
  # this can make Automake try to regenerate configure/aclocal.m4 during make,
  # which requires the obsolete aclocal-1.14. These files are shipped with the
  # vendored release and must be used as-is for an offline build.
  ExternalProject_Add_Step(glog_src prevent_autoreconf
    COMMAND ${CMAKE_COMMAND} -E touch
      ${GLOG_SOURCE_DIR}/aclocal.m4
      ${GLOG_SOURCE_DIR}/configure
      ${GLOG_SOURCE_DIR}/Makefile.in
      ${GLOG_SOURCE_DIR}/src/config.h.in
    DEPENDEES configure
    DEPENDERS build
    ALWAYS 1
  )

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/glog
          DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
          FILES_MATCHING PATTERN "*.h")
  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
          DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          FILES_MATCHING PATTERN "libglog*")
  cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
            LIBRARIES glog)
else()
  message(STATUS "Using system Glog.")
  find_package(Glog REQUIRED)
  cs_install()
  cs_export(INCLUDE_DIRS  ${GLOG_INCLUDE_DIRS}
            LIBRARIES     ${GLOG_LIBRARIES})
endif()
