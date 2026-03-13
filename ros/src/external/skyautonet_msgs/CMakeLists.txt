cmake_minimum_required(VERSION 3.5)
project(skyautonet_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_common_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

set(msg_files
  "msg/ActuatorState.msg"
  "msg/ActuatorParam.msg"
  "msg/ObdParam.msg"
  "msg/ObdMCU.msg"
  "msg/ObdBMS.msg"
  "msg/ObdOBC.msg"
  "msg/ObdDC.msg"
  "msg/ObdEPS.msg"
  "msg/ObdTPMS.msg"
  "msg/ObdCHG.msg"
  "msg/PlanningMode.msg"
  "msg/RoutePoseArr.msg"
  "msg/VehicleInformation.msg"
  "msg/ControlMode.msg"
  "msg/ControlCleaning.msg"
  "msg/ControllerDebug.msg"
  "msg/CleaningState.msg"
  "msg/Object2D.msg"
  "msg/Object3D.msg"
  "msg/ObjectArray.msg"
  "msg/SensorStatus.msg"
  "msg/SensorStatusArr.msg"
  "msg/DiagStatus.msg"
  "msg/DiagStatusArr.msg"
  "msg/DummyInfo.msg"
  "msg/ManualControl.msg"
  "msg/DrivingStatus.msg"
  "msg/Gphdt.msg"
  "msg/DigitalTachoGraph.msg"
  "msg/CollisionWarning.msg"
  "msg/CheckList.msg"
  "msg/ServerCommand.msg"
  "msg/SoundPlay.msg"
  "msg/Fod.msg"
  "msg/FodArray.msg"
  "msg/Bird.msg"
  "msg/BirdObject.msg"
  "msg/BirdSpaObject.msg"
  "msg/ForceFeedback.msg"
  "msg/FodPlanningStatus.msg" 
  "msg/LaneDist.msg"
  "srv/ChangePlanning.srv"
  "srv/Engage.srv"
  "srv/GetPathPoints.srv"
  "srv/CheckFod.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs autoware_planning_msgs
 )
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
