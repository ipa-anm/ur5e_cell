#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ur5e_cell_manipulation/plan_motion_node.hpp"
#include "ur5e_cell_manipulation/execute_motion_node.hpp"
#include "ur5e_cell_manipulation/gripper_node.hpp"
#include "ur5e_cell_manipulation/set_pose_node.hpp"
#include "ur5e_cell_manipulation/detect_aruco_node.hpp"
#include "ur5e_cell_manipulation/detect_color_node.hpp"
#include "ur5e_cell_manipulation/manipulation_color_detection.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"




ManipulationBehaviorManager::ManipulationBehaviorManager(rclcpp::Node::SharedPtr node, std::string planning_group)
  : node_(node), planning_group_(planning_group)
{
  moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
  moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
  planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group, moveit_cpp_);
  factory_ = std::make_shared<BT::BehaviorTreeFactory>();
  node_->declare_parameter(
      "behavior_file",
      ament_index_cpp::get_package_share_directory("ur5e_cell_manipulation").append("/config/behavior.xml"));
}
void ManipulationBehaviorManager::start_behavior()
{
  RCLCPP_INFO(node_->get_logger(), "Starting behavior");
  config_ = new BT::NodeConfiguration();
  config_->blackboard = BT::Blackboard::create();
  config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
  config_->blackboard->set<moveit_cpp::MoveItCppPtr>("moveit_cpp", moveit_cpp_);
  config_->blackboard->set<moveit_cpp::PlanningComponentPtr>("planning_component", planning_component_);
  config_->blackboard->set<std::string>("planning_group", planning_group_);
  RCLCPP_INFO(node_->get_logger(), "Set blackboard");
  BT::NodeBuilder plan_builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<PlanMotionNode>(name, config);
  };
  BT::NodeBuilder exec_builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<ExecuteMotionNode>(name, config);
  };
  BT::NodeBuilder gripper_builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<GripperActionNode>(name, config);
  };
  BT::NodeBuilder set_pose_builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<SetPoseNode>(name, config);
  };
  BT::NodeBuilder aruco_detector_builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<DetectArucoNode>(name, config);
  };
  BT::NodeBuilder color_detector_builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<DetectColorNode>(name, config);
  };

  try
  {
    factory_->registerBuilder<PlanMotionNode>("PlanMotion", plan_builder);
    factory_->registerBuilder<ExecuteMotionNode>("ExecuteMotion", exec_builder);
    factory_->registerBuilder<GripperActionNode>("GripperAction", gripper_builder);
    factory_->registerBuilder<SetPoseNode>("SetPose", set_pose_builder);
    factory_->registerBuilder<DetectArucoNode>("GetPickFromAruco", aruco_detector_builder);
    factory_->registerBuilder<DetectColorNode>("GetPickFromColor", color_detector_builder);
  }
  catch (std::exception& e)
  {
    RCLCPP_INFO(node_->get_logger(), "Exception: %s", e.what());
  }
  RCLCPP_INFO(node_->get_logger(), "Registered Builders");
  std::string file_name = node_->get_parameter("behavior_file").as_string();
  RCLCPP_INFO(node_->get_logger(), "Loading behavior from %s", file_name.c_str());
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromFile(file_name, config_->blackboard));
  BT::PublisherZMQ publisher_zmq(*tree_);
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS ||
         tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
    rclcpp::sleep_for(std::chrono::milliseconds(1));
  }
}
