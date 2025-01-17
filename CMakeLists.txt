cmake_minimum_required(VERSION 3.16)
project(mcts-path-planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -Wunused -Wshadow -O1 -g1")
set(CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=native -flto")

if(NOT CMAKE_BUILD_TYPE)
  message(
    STATUS
      "CMAKE_BUILD_TYPE not set, setting to Debug. Set with -DCMAKE_BUILD_TYPE=<type>"
  )
  set(CMAKE_BUILD_TYPE Debug)
else()
  message(STATUS "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
endif()

find_program(CCACHE ccache)
if(CCACHE)
  set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
  message(STATUS "Using ccache for compilation: ${CCACHE}")
else()
  message(WARNING "ccache not found - install ccache to speed up compilation")
endif()

find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

qt_standard_project_setup()

qt_add_executable(mcts
  src/main.cc
  src/mcts.cc
  src/grid.cc
)

target_link_libraries(mcts PRIVATE
    Qt::Core
    Qt::Widgets
)
