cmake_minimum_required(VERSION 3.11)
project(gaden_preprocessing)



set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


set(CMAKE_CXX_STANDARD 20)

# Ament dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()


# Main Target
ament_auto_add_executable(preprocessing src/preprocessing.cpp)
 

# configureCoppeliaSim: moves the robot's start pose in a running CoppeliaSim scene via the
# ZMQ Remote API C++ client. That client ships inside the CoppeliaSim SDK itself (not on the
# ROS2/ament index), so it needs its own find/fetch here rather than a plain ament_target_dependencies.
if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
  set(COPPELIASIM_ZMQ_CPP_CLIENT_DIR "$ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp")
endif()

if(COPPELIASIM_ZMQ_CPP_CLIENT_DIR AND EXISTS "${COPPELIASIM_ZMQ_CPP_CLIENT_DIR}/RemoteAPIClient.h")
  include(FetchContent)
  FetchContent_Declare(jsoncons
    GIT_REPOSITORY https://github.com/danielaparker/jsoncons
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/jsoncons
  )
  FetchContent_GetProperties(jsoncons)
  if(NOT jsoncons_POPULATED)
    FetchContent_Populate(jsoncons)
  endif()

  find_library(ZMQ_LIBRARY zmq REQUIRED)
  find_package(fmt REQUIRED)

  add_executable(configureCoppeliaSim
    src/configureCoppeliaSim.cpp
    "${COPPELIASIM_ZMQ_CPP_CLIENT_DIR}/RemoteAPIClient.cpp"
  )
  target_include_directories(configureCoppeliaSim PRIVATE
    "${COPPELIASIM_ZMQ_CPP_CLIENT_DIR}"
    "${jsoncons_SOURCE_DIR}/include"
  )
  target_compile_definitions(configureCoppeliaSim PRIVATE GENERATE_COPPELIA_SCENE SIM_REMOTEAPICLIENT_OBJECTS)
  target_link_libraries(configureCoppeliaSim PRIVATE ${ZMQ_LIBRARY} fmt::fmt)
  ament_target_dependencies(configureCoppeliaSim PUBLIC rclcpp geometry_msgs tf2_geometry_msgs)

  install(TARGETS configureCoppeliaSim DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "COPPELIASIM_ROOT_DIR not set or zmqRemoteApi cpp client not found; skipping configureCoppeliaSim (main_simbot.py needs it)")
endif()

# Install
install(
  TARGETS preprocessing 
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()