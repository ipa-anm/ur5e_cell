#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

class ManipulationBehaviorManager
{
protected:
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  rclcpp::Node::SharedPtr node_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  std::string planning_group_;
  BT::NodeConfiguration* config_;
  std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  std::shared_ptr<BT::Tree> tree_;

public:
  ManipulationBehaviorManager(rclcpp::Node::SharedPtr node, std::string planning_group);
  void start_behavior();
};
