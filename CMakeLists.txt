#
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(libcaer_driver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(libcaer REQUIRED)

#
# --------- driver (composable component) -------------

add_library(driver SHARED  src/libcaer_wrapper.cpp  src/driver.cpp)
ament_target_dependencies(driver rclcpp rclcpp_components event_camera_msgs std_srvs)
target_include_directories(driver PUBLIC include)
target_link_libraries(driver libcaer::caer)

rclcpp_components_register_nodes(driver "libcaer_driver::Driver")

# --------- driver (plain old node) -------------
add_executable(driver_node src/driver_node.cpp)
target_link_libraries(driver_node driver)


# the node must go into the project specific lib directory or else
# the launch file will not find it
install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  driver
  DESTINATION lib)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  # find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_flake8(--config ${CMAKE_CURRENT_SOURCE_DIR}/.flake8.ini)
  #ament_lint_cmake(--filter=-readability/wonkycase)
  ament_lint_cmake()
  # ament_pep257()   # not working on galactic / foxy
  ament_xmllint()
  ament_clang_format(CONFIG_FILE .clang-format)
endif()

ament_package()
