cmake_minimum_required(VERSION 3.8)
project(adnav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


# Declare messages
set(msg_files
  "msg/ANPPHeader.msg"
  "msg/FilterStatus.msg"
  "msg/GNSSFixStatus.msg"
  "msg/LLH.msg"
  "msg/NED.msg"
  "msg/RPH.msg"
  "msg/RawStatusPacket.msg"
  "msg/SystemStatus.msg"
  "msg/RawAcknowledge.msg"
  "msg/PacketPeriod.msg"
  "msg/DeviceInformationPacket.msg"
  "msg/RequestPacket.msg"
  "msg/SerialInterface.msg"
)

set(srv_files
  "srv/PacketTimerPeriod.srv"
  "srv/PacketPeriods.srv"
  "srv/RequestPacket.srv"
  "srv/Ntrip.srv"
)

# Build the message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces# Packages that above messages depend on
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()


