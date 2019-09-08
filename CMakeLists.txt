cmake_minimum_required(VERSION 2.8.3)
project(fximu)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosserial_client
  rosserial_tivac
  std_msgs
)

add_message_files(
  FILES SensorData.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_tivac
  SCRIPT make_libraries_tiva
)

add_dependencies(${PROJECT_NAME}_ros_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})

rosserial_configure_client(
  DIRECTORY ros_fximu
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(ros_fximu ros_fximu.axf ALL)
rosserial_add_client_target(ros_fximu flash)
rosserial_add_client_target(ros_fximu size)

rosserial_configure_client(
  DIRECTORY calibration_firmware
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(calibration_firmware calibration_firmware.axf ALL)
rosserial_add_client_target(calibration_firmware flash)
rosserial_add_client_target(calibration_firmware size)