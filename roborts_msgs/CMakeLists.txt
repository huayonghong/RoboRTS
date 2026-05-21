cmake_minimum_required(VERSION 3.5)
project(roborts_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TwistAccel.msg"
  "msg/GimbalAngle.msg"
  "msg/GimbalRate.msg"
  "msg/ObstacleMsg.msg"
  "msg/ShootInfo.msg"
  "msg/ShootState.msg"
  "msg/BonusStatus.msg"
  "msg/GameResult.msg"
  "msg/GameStatus.msg"
  "msg/GameSurvivor.msg"
  "msg/ProjectileSupply.msg"
  "msg/RobotBonus.msg"
  "msg/RobotDamage.msg"
  "msg/RobotHeat.msg"
  "msg/RobotShoot.msg"
  "msg/RobotStatus.msg"
  "msg/SupplierStatus.msg"
  "srv/GimbalMode.srv"
  "srv/FricWhl.srv"
  "srv/ShootCmd.srv"
  "action/GlobalPlanner.action"
  "action/LocalPlanner.action"
  "action/ArmorDetection.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs action_msgs
)

ament_package()
