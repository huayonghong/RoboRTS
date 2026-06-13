/***************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 ***************************************************************************/

#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/make_shared.hpp>
#include <boost/pointer_cast.hpp>

#include <functional>

#include "timed_elastic_band/teb_optimal.h"
#include "timed_elastic_band/proto/timed_elastic_band.pb.h"

#include "local_planner/optimal_base.h"
#include "local_planner/obstacle.h"
#include "local_planner/data_converter.h"
#include "local_planner/local_visualization.h"
#include "local_planner/robot_footprint_model.h"

using namespace roborts_local_planner;

namespace {

OptimalBasePtr planner;
std::vector<ObstaclePtr> obst_vector;
LocalVisualizationPtr visual;
ViaPointContainer via_points;
unsigned int no_fixed_obstacles = 0;

void CB_obstacle_marker(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> feedback);

void CreateInteractiveMarker(
    const double &init_x, const double &init_y, unsigned int id, std::string frame,
    const rclcpp::Time &stamp,
    interactive_markers::InteractiveMarkerServer *marker_server,
    interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb) {
  visualization_msgs::msg::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = stamp;
  std::ostringstream oss;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;

  visualization_msgs::msg::Marker box_marker;
  box_marker.type = visualization_msgs::msg::Marker::CUBE;
  box_marker.id = static_cast<int32_t>(id);
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 1.0F;
  box_marker.color.g = 0.5F;
  box_marker.color.b = 0.5F;
  box_marker.color.a = 1.0F;

  visualization_msgs::msg::InteractiveMarkerControl box_control;
  box_control.always_visible = 1;
  box_control.markers.push_back(box_marker);

  i_marker.controls.push_back(box_control);

  visualization_msgs::msg::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = static_cast<float>(sqrt(2) / 2);
  move_control.orientation.x = 0;
  move_control.orientation.y = static_cast<float>(sqrt(2) / 2);
  move_control.orientation.z = 0;
  move_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

  i_marker.controls.push_back(move_control);

  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name, feedback_cb);
}

class TebTestNode : public rclcpp::Node {
 public:
  TebTestNode() : Node("test_optim_node") {
    std::string full_path = ament_index_cpp::get_package_share_directory("roborts_planning") +
                           "/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt";
    roborts_common::ReadProtoFromTextFile(full_path.c_str(), &param_config_);

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "marker_obstacles", shared_from_this());

    obst_vector.emplace_back(boost::make_shared<PointObstacle>(-3, 1));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(6, 2));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(0, 0.1));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(-4, 1));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(5, 2));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(1, 0.1));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(-3, 2));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(5, 3));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(4, 0));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(4, 1));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(3, 2));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(2, 2));

    for (unsigned int i = 0; i < obst_vector.size(); ++i) {
      boost::shared_ptr<PointObstacle> pobst =
          boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
      if (pobst) {
        CreateInteractiveMarker(pobst->Position().coeff(0), pobst->Position().coeff(1), i,
                                "odom", now(), marker_server_.get(),
                                &CB_obstacle_marker);
      }
    }
    marker_server_->applyChanges();

    visual = std::make_shared<LocalVisualization>(shared_from_this(), "odom");

    RobotFootprintModelPtr model = boost::make_shared<PointRobotFootprint>();
    planner = OptimalBasePtr(new TebOptimal(param_config_, &obst_vector, model, visual, &via_points));

    no_fixed_obstacles = static_cast<unsigned int>(obst_vector.size());

    cycle_timer_ = create_wall_timer(std::chrono::milliseconds(25),
                                       std::bind(&TebTestNode::OnMainCycle, this));
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                        std::bind(&TebTestNode::OnPublishCycle, this));
  }

 private:
  void OnMainCycle() {
    auto start_pose = DataConverter::LocalConvertCData(-4, 0, 0);
    auto end_pose = DataConverter::LocalConvertCData(4, 0, 0);
    planner->Optimal(DataBase(start_pose.first, start_pose.second), DataBase(end_pose.first, end_pose.second));
  }

  void OnPublishCycle() { planner->Visualize(); }

  Config param_config_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  rclcpp::TimerBase::SharedPtr cycle_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

void CB_obstacle_marker(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> feedback) {
  std::stringstream ss(feedback->marker_name);
  unsigned int index = 0;
  ss >> index;
  if (index >= no_fixed_obstacles) {
    return;
  }
  PointObstacle *pobst = dynamic_cast<PointObstacle *>(obst_vector.at(index).get());
  if (pobst) {
    pobst->Position() = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
  }
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TebTestNode>());
  rclcpp::shutdown();
  return 0;
}
