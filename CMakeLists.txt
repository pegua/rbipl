#CMakeLists.txt inspired by KDL's CMakeLists.txt
#and CMake tutorial: http://www.cmake.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file

###################################################
#                                                 #
#       Start project customization section       #
#                                                 #
###################################################

cmake_minimum_required(VERSION 2.6)

PROJECT(RBIPL)

set( RBIPL_VERSION_MAJOR 0)
set( RBIPL_VERSION_MINOR 0)
set( RBIPL_VERSION_PATCH 1)
set( RBIPL_VERSION
  ${RBIPL_MAJOR_VERSION}.${RBIPL_MINOR_VERSION}.${RBIPL_PATCH_VERSION})
  
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Orocos-KDL REQUIRED)

include_directories(${Orocos-KDL_INCLUDE_DIR})



# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/RBIPL)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")
 
# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()
 
# set up include-directories
include_directories(
  "${PROJECT_SOURCE_DIR}"   # to find foo/foo.h
  "${PROJECT_BINARY_DIR}")  # to find foo/config.h
 
# Add sub-directories
add_subdirectory(src)
add_subdirectory(tests)
 
# The interesting stuff goes here
# ===============================
 
# Add all targets to the build-tree export set
export(TARGETS rbipl test_chaininertialparameters
  FILE "${PROJECT_BINARY_DIR}/RBIPLTargets.cmake")
 
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE RBIPL)
 
# Create the FooBarConfig.cmake and FooBarConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(RBIPLConfig.cmake.in
  "${PROJECT_BINARY_DIR}/RBIPLConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${RBIPL_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(RBIPLConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RBIPLConfig.cmake" @ONLY)
# ... for both
configure_file(RBIPLConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/RBIPLConfigVersion.cmake" @ONLY)
 
# Install the FooBarConfig.cmake and FooBarConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RBIPLConfig.cmake"
  "${PROJECT_BINARY_DIR}/RBIPLConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
 
# Install the export set for use with the install-tree
install(EXPORT RBIPLTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)
