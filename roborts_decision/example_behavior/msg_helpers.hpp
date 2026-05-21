/****************************************************************************
 * Geometry helpers for decision behaviors (tf2 equivalents of tf::* helpers).
 ***************************************************************************/
#ifndef ROBORTS_DECISION_MSG_HELPERS_HPP
#define ROBORTS_DECISION_MSG_HELPERS_HPP

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

namespace roborts_decision {

inline geometry_msgs::msg::Quaternion QuaternionFromRollPitchYaw(double roll, double pitch,
                                                                 double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

inline geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw) {
  return QuaternionFromRollPitchYaw(0.0, 0.0, yaw);
}

inline double YawFromOrientation(const geometry_msgs::msg::Quaternion &orientation) {
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  return tf2::getYaw(q);
}

} // namespace roborts_decision

#endif // ROBORTS_DECISION_MSG_HELPERS_HPP
