cmake_minimum_required(VERSION 3.10...3.20)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

# Project Name
project(GBPPlanner)

# Definitions and package finding
add_compile_definitions(EIGEN_DONT_PARALLELIZE)
find_package(OpenMP REQUIRED)
add_subdirectory(thirdparty/raylib)

#################################################################

# GBP Library
add_library(gbp STATIC
    src/gbp/Factor.cpp 
    src/gbp/Variable.cpp 
    src/gbp/FactorGraph.cpp 
    
)
target_compile_definitions(gbp PUBLIC ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/")
target_include_directories(gbp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
target_link_libraries(gbp PUBLIC OpenMP::OpenMP_CXX raylib)

# Robot Library
add_library(robot STATIC
    src/Robot.cpp
)
target_include_directories(robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(robot PUBLIC gbp)

# Graphics Library
add_library(graphics STATIC
    src/Graphics.cpp
)
target_include_directories(graphics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(graphics PUBLIC gbp)

# Task Library
add_library(task STATIC
    src/Task.cpp
)
target_include_directories(task PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(task PUBLIC gbp)

# Simulator Executable
add_executable(gbpplanner 
    src/main.cpp
    src/Globals.cpp
    src/Utils.cpp 
    src/Simulator.cpp
)
target_include_directories(gbpplanner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(gbpplanner PUBLIC gbp robot graphics task)

# Platform specific configurations
if (APPLE)
    target_link_libraries(
      gbpplanner PUBLIC
          "-framework Cocoa" "-framework OpenGL" "-framework IOKit" "-framework CoreVideo"
      )
endif()

if (WIN32)
    target_link_libraries(
      gbpplanner PUBLIC
          OpenMP::OpenMP_CXX
          raylib
          gdi32
          winmm
          imm32
          ole32
          oleaut32
          version
          uuid
      )
endif()

#################################################################
