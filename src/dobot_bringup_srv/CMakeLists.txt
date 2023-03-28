cmake_minimum_required(VERSION 3.5)
project(dobot_bringup_srv)

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

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AccJ.srv"
  "srv/AccL.srv"
  "srv/AO.srv"
  "srv/AOExecute.srv"
  "srv/Arc.srv"
  "srv/Arch.srv"
  "srv/Circle.srv"
  "srv/ClearError.srv"
  "srv/ContinueScript.srv"
  "srv/CP.srv"
  "srv/DisableRobot.srv"
  "srv/DO.srv"
  "srv/DOExecute.srv"
  "srv/EmergencyStop.srv"
  "srv/EnableRobot.srv"
  "srv/GetHoldRegs.srv"
  "srv/JointMovJ.srv"
  "srv/Jump.srv"
  "srv/LimZ.srv"
  "srv/MoveJog.srv"
  "srv/MovJ.srv"
  "srv/MovL.srv"
  "srv/PauseScript.srv"
  "srv/PayLoad.srv"
  "srv/PowerOn.srv"
  "srv/RelMovJ.srv"
  "srv/RelMovL.srv"
  "srv/ResetRobot.srv"
  "srv/RobotMode.srv"
  "srv/RunScript.srv"
  "srv/ServoJ.srv"
  "srv/ServoP.srv"
  "srv/SetArmOrientation.srv"
  "srv/SetCollisionLevel.srv"
  "srv/SetHoldRegs.srv"
  "srv/SetObstacleAvoid.srv"
  "srv/SetSafeSkin.srv"
  "srv/SpeedFactor.srv"
  "srv/SpeedJ.srv"
  "srv/SpeedL.srv"
  "srv/StartFCTrace.srv"
  "srv/StartPath.srv"
  "srv/StartTrace.srv"
  "srv/StopScript.srv"
  "srv/Sync.srv"
  "srv/Tool.srv"
  "srv/ToolDO.srv"
  "srv/ToolDOExecute.srv"
  "srv/User.srv"
)

ament_package()
