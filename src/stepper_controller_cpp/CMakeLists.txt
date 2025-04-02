# ... (previous CMakeLists.txt targeting libgpiodcxx) ...
cmake_minimum_required(VERSION 3.8)
project(stepper_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find libgpiodcxx (C++ bindings) using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIODCXX REQUIRED IMPORTED_TARGET libgpiodcxx) # Use CXX bindings

if(NOT GPIODCXX_FOUND)
    message(FATAL_ERROR "libgpiodcxx not found via pkg-config. Please install libgpiod-dev or build from source with C++ bindings.")
else()
    message(STATUS "libgpiodcxx v${GPIODCXX_VERSION} found.") # Should show 2.2.1
endif()

add_executable(motor_controller_node src/motor_controller_node_main.cpp
                                     src/motor_controller_node.cpp
                                     src/gpiod_stepper_motor.cpp)

target_include_directories(motor_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(motor_controller_node
  rclcpp
  std_msgs)

# Add C++ include dirs (pkg-config will provide /usr/local/include)
target_include_directories(motor_controller_node PRIVATE ${GPIODCXX_INCLUDE_DIRS})
# Link against C++ library target (pkg-config will provide /usr/local/lib)
target_link_libraries(motor_controller_node PkgConfig::GPIODCXX)
message(STATUS "Linking against libgpiodcxx.")

install(TARGETS
  motor_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()