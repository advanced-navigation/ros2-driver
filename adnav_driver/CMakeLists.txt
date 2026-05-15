cmake_minimum_required(VERSION 3.28)
project(adnav_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Avoid Confusing max() and min() for Windows Macros. See rclcpp issue #1054
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(adnav_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(uvw REQUIRED)

include_directories(
  ../an-ros-common/include
)

aux_source_directory(src src_MAIN)
aux_source_directory(../an-ros-common/src src_COMMON)

add_library(${PROJECT_NAME}_lib
  ${src_MAIN}
  ${src_COMMON}
)

set(DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "diagnostic_updater"
  "geometry_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "adnav_interfaces"
  "geographic_msgs"
)

set(EXECUTABLE_NAME "adnav_driver")

generate_parameter_library(
        adnav_parameters
        src/adnav_parameters.yaml
)

target_link_libraries(${PROJECT_NAME}_lib PUBLIC
        ${adnav_interfaces_TARGETS}
        ${geographic_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${std_srvs_TARGETS}
        diagnostic_updater::diagnostic_updater
        geographic_msgs::validation
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs_library
        tf2::tf2
        tf2_geometry_msgs::tf2_geometry_msgs
        uvw::uvw
        adnav_parameters
)

add_executable(${EXECUTABLE_NAME} src/main.cpp)
target_link_libraries(${EXECUTABLE_NAME}
        ${PROJECT_NAME}_lib
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  ament_cpplint()
endif()

ament_package()
