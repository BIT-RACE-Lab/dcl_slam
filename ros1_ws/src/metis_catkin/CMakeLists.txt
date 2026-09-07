cmake_minimum_required(VERSION 2.8.3)
project(metis_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

set(METIS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/metis" CACHE PATH "Path to the local METIS source tree")
if(NOT EXISTS "${METIS_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "METIS source was not found at: ${METIS_SOURCE_DIR}")
endif()
set(METIS_CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE:STRING=Release -DSHARED:BOOL=TRUE -DGKLIB_PATH:STRING=${METIS_SOURCE_DIR}/GKlib)

# The following logic addresses a potential Xcode bug.
# It is documented at
#  https://www.mail-archive.com/petsc-users@mcs.anl.gov/msg28472.html
# and uses gcc as workaround.
# Consider removal when updating to a new metis version.
if( (CMAKE_C_COMPILER MATCHES "(.*)Xcode(.*)") AND (CMAKE_C_COMPILER_VERSION MATCHES "7.3(.*)") )
  message("Use of XCode infeasible due to potential compiler bug. Switching to gcc.")
  ExternalProject_Add(metis_src
    SOURCE_DIR "${METIS_SOURCE_DIR}"
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${CMAKE_SOURCE_DIR}/gcc.cmake ${METIS_CMAKE_ARGS}
    BUILD_COMMAND $(MAKE)
    INSTALL_COMMAND $(MAKE) install
  )
else() 
  ExternalProject_Add(metis_src
    SOURCE_DIR "${METIS_SOURCE_DIR}"
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    CMAKE_ARGS ${METIS_CMAKE_ARGS} 
    BUILD_COMMAND $(MAKE)
    INSTALL_COMMAND $(MAKE) install
  ) 
endif()

install(FILES ${CATKIN_DEVEL_PREFIX}/include/metis.h
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libmetis*")
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES metis)
