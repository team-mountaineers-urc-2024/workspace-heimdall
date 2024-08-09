cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanCommand.msg"
  "msg/ManipulatorControl.msg"
  "msg/MotorData.msg"
  "msg/MyActuatorInfo.msg"
  "msg/Status.msg"
  "msg/UrcCustomPath.msg"
  "msg/UrcCustomPoint.msg"
  "msg/WaypointObject.msg"
  "msg/EDWaypoint.msg"
  "msg/EDWaypointList.msg"
  "srv/AddXYZ.srv"
  "srv/Clear.srv"
  "srv/CollectSample.srv"
  "srv/CollectSpectrometerData.srv"
  "srv/GeneratePoints.srv"
  "srv/GeneratePointsAround.srv"
  "srv/GetDistanceValue.srv"
  "srv/GoToNextPoint.srv"
  "srv/GoToPointSrv.srv"
  "srv/GUIWaypointPath.srv"
  "srv/Heartbeat.srv"
  "srv/MoistureSensor.srv"
  "srv/ObjectPosition.srv"
  "srv/Query.srv"
  "srv/QueryWaypoint.srv"
  "srv/RebootScoop.srv"
  "srv/Remove.srv"
  "srv/Reorder.srv"
  "srv/Replace.srv"
  "srv/SendCanCommand.srv"
  "srv/SendScoopHome.srv"
  "srv/SetPIDPlannerPrecision.srv"
  "srv/SpinCentrifuge.srv"
  "srv/SubsurfaceMotor.srv"
  "srv/Waypoint.srv"
  "action/GoToPoint.action"
  DEPENDENCIES std_msgs geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
