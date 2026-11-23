# **********************************************************************************************************************
 # Copyright (c) 2025 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 # 
 # Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 # following conditions are met:
 # 
 # 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 #    disclaimer.
 # 
 # 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 #    following disclaimer in the documentation and/or other materials provided with the distribution.
 # 
 # 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 #    products derived from this software without specific prior written permission.
 # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 # INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 # SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 # WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# **********************************************************************************************************************

#--------------------------------------------------------------
#  
# This minimal sample project can be used as a template for a
# user project.  Modify sections 1, 2, and 3 below as appropriate.
# 
#--------------------------------------------------------------
 

cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

#--------------------------------------------------------------
# === 1 === 
# Set the project name
#--------------------------------------------------------------

project(acsl_physics_sim)

#--------------------------------------------------------------
# === 2 ===
# Find the Chrono package and any REQUIRED or OPTIONAL modules
# by invoking the find_package function in CONFIG mode:
#    find_package(Chrono
#                 COMPONENTS req_module1 req_module1 ...
#                 OPTIONAL_COMPONENTS opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono modules can be requested (case insensitive):
#   Cascade, Cosimulation, Irrlicht, OpenGL, Matlab, Multicore, Gpu,
#   PardisoMKL, PardisoProject, Postprocess, Python, Vehicle,
#   VehicleCosimm, VSG.
# A component can be requested either as required or optional
# (see the CMake documentation for find_package).
# 
# Note that you will have to set the variable Chrono_DIR to 
# specify the location of the chrono-config.cmake script, if
# it is not in its default install location.
# Chrono_DIR can be either a Chrono build tree or a Chrono install tree.
# 
# The following variables are set and can be used further down:
# Chrono_FOUND
#   set to true if Chrono and all required components were found
# CHRONO_C_FLAGS
# CHRONO_CXX_FLAGS
#   C and C++ compilation flags
# CHRONO_INCLUDE_DIRS
#   additional paths for included headers
# CHRONO_LIBRARIES
#   list of required libraries (with full path)
# CHRONO_LINKER_FLAGS
#   additional linker flags
# CHRONO_DATA_DIR
#   path to the Chrono data make_directory
# 
# In addition, for each requested component [COMPONENT], the
# following variable is set to true (ON) or false (OFF):
# CHRONO_[COMPONENT]_FOUND
# 
# In this example, we only request the Irrlicht module (required)
# and, for demonstration purposes, the PardisoMKL module (optional)
#--------------------------------------------------------------

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")

# Set default path for the chrono_dir and cache it for GUI visibility
if(NOT Chrono_DIR)
  set(Chrono_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libraries/chrono-build/cmake" CACHE PATH "Chrono config directory")
endif()

# NOTE: vsg, vsgXchange, and vsgImGui are installed globally to /usr/local by installer.sh.
# CMake's find_package(... CONFIG) automatically searches /usr/local/lib/cmake/<pkg>/ as
# part of its default system prefix path, so no vsg_DIR/vsgXchange_DIR/vsgImGui_DIR
# overrides are needed here anymore.

# Set default build type to Release and cache it for GUI visibility
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
endif()

# Find pacakges needed for the codebase
#   - Most systems have 1.71.0. Therfore it is set as the minimum required version of Boost
find_package(Boost 1.71 REQUIRED COMPONENTS log log_setup system filesystem thread)

find_package(Chrono
             COMPONENTS Irrlicht VSG
             OPTIONAL_COMPONENTS PardisoMKL
             CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

if (NOT Boost_FOUND)
  message("Could not find Boost or one of its required components")
  return()
endif()


#--------------------------------------------------------------
# Important! To ensure ABI compatibility, use the same C++ standard
# as the one used to build the Chrono libraries.
#--------------------------------------------------------------

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/third-party/fkYAML/single_include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/third-party/nlohmann-json/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/third-party/rncarpio-linterp/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/third-party/d99kris-rapidcsv/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-utilities)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-main)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-system)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-tailsitter)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-quadm)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-x8)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-scene)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/sim-scene)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-scene/sim-locales)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-scene/sim-locales/sim-flat)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-bridge)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-trajectory)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-trajectory/sim-trajectories)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-trajectory/sim-trajectories/piecewise-polynomial)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-trajectory/sim-trajectories/vector-interpolation)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-aerofoil)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-aerofoil/sim-aerofoils/NACA0012)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp/controllers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp/controllers/pid-omega)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp/controllers/mrac-omega)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp/controllers/mrac-observer)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp/controllers/mrac-long-lat)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp/controllers/pid-quaternion)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp/controllers/pid-geometric)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-qrbp/controllers/mrac-geometric)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-tailsitter/controllers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-tailsitter/controllers/mrac-geometric)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-tailsitter/controllers/mrac-hybrid)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-quadm/controllers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-quadm/controllers/mrac-omega)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-quadm/controllers/mrac-geometric)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-quadm/controllers/mrac-geometric-na-ebci)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-x8/controllers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-x8/controllers/mrac-omega)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim-uav/sim-platforms/sim-x8/controllers/mrac-geometric)

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
endif()

#--------------------------------------------------------------
# === 3 ===
# Add the executable from your project and specify all C++ 
# files in your project. 
#--------------------------------------------------------------
set(SOURCES
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-main/sim-main.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-system/sim-system.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-scene/sim-locales/sim-flat/sim-flat.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-bridge/sim-bridge.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-aerofoil/sim-aerofoil.cpp

   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-utilities/sim-helpers.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-utilities/sim-logger.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-utilities/continuous-lyapunov-equation.cpp

   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-qrbp/sim-qrbp.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-tailsitter/sim-tailsitter.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-quadm/sim-quadm.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-x8/sim-x8.cpp

   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-trajectory/sim-trajectories/piecewise-polynomial/piecewise-polynomial-trajectory.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-trajectory/sim-trajectories/vector-interpolation/vector-interpolation-trajectory.cpp
   
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-qrbp/controllers/pid-omega/pid-omega-qrbp.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-qrbp/controllers/mrac-omega/mrac-omega-qrbp.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-qrbp/controllers/mrac-observer/mrac-observer-qrbp.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-qrbp/controllers/mrac-long-lat/mrac-long-lat-qrbp.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-qrbp/controllers/pid-quaternion/pid-quaternion-qrbp.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-qrbp/controllers/pid-geometric/pid-geometric-qrbp.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-qrbp/controllers/mrac-geometric/mrac-geometric-qrbp.cpp

   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-tailsitter/controllers/mrac-geometric/mrac-geometric-tailsitter.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-tailsitter/controllers/mrac-hybrid/mrac-hybrid-tailsitter.cpp

   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-quadm/controllers/mrac-omega/mrac-omega-quadm.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-quadm/controllers/mrac-geometric/mrac-geometric-quadm.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-quadm/controllers/mrac-geometric-na-ebci/mrac-geometric-na-ebci-quadm.cpp

   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-x8/controllers/mrac-omega/mrac-omega-x8.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/sim-uav/sim-platforms/sim-x8/controllers/mrac-geometric/mrac-geometric-x8.cpp
   )

add_executable(acsl_sim ${SOURCES})

#--------------------------------------------------------------
# Set properties for your executable target
# 
# Note that here we define a macro CHRONO_DATA_DIR which will
# contain the path to the Chrono data directory, either in its
# source tree (if using a build version of Chrono), or in its
# install tree (if using an installed version of Chrono).
#--------------------------------------------------------------

target_compile_definitions(acsl_sim PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(acsl_sim PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(acsl_sim PUBLIC ${CHRONO_LINKER_FLAGS})

#--------------------------------------------------------------
# Link to Chrono libraries and dependency libraries
#--------------------------------------------------------------

target_link_libraries(acsl_sim
    ${CHRONO_LIBRARIES}
    ${Boost_LOG_LIBRARY}
    ${Boost_LOG_SETUP_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# DLLs will be copied into ${PROJECT_BINARY_DIR}/${config} by default
# or in ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${config} if only CMAKE_RUNTIME_OUTPUT_DIRECTORY is set
# or to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG>} if the specific CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG> has been set
#--------------------------------------------------------------

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "<mycustompathforrelease>")
add_DLL_copy_command()