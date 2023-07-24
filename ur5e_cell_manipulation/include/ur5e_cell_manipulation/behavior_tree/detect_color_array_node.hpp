#ifndef DETECT_COLOR_NODE_HPP
#define DETECT_COLOR_NODE_HPP

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <color_pose_msgs/msg/color_pose_array.hpp>
#include <color_pose_msgs/msg/color_pose.hpp>

#include <atomic>
#include <iostream>
#include <fstream>
const std::string color_pose_names[16] = {
  "pick_green_state",  "pre_green_state",  "release_green_state",  "prerelease_green_state",
  "pick_blue_state",   "pre_blue_state",   "release_blue_state",   "prerelease_blue_state",
  "pick_red_state",    "pre_red_state",    "release_red_state",    "prerelease_red_state",
  "pick_yellow_state", "pre_yellow_state", "release_yellow_state", "prerelease_yellow_state",
};

class DetectColorNode : public BT::StatefulActionNode
{
protected:
  bool online_;
  int required_poses_ = 5;
  int poses_counter = 5;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  color_pose_msgs::msg::ColorPoseArray final_pose_array;
  std::atomic_int counter_;
  std::atomic_bool error_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  std::string planning_group_;
  rclcpp::Subscription<color_pose_msgs::msg::ColorPoseArray>::SharedPtr pose_array;

public:
  DetectColorNode(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Initialising Execute Node.");
    sleep(1);
    pose_array = node_->create_subscription<color_pose_msgs::msg::ColorPoseArray>(
        "color_pose_estimation/color_pose_array", 30,
        std::bind(&DetectColorNode::getPoseCallback, this, std::placeholders::_1));
    moveit_cpp_ = config.blackboard->get<moveit_cpp::MoveItCppPtr>("moveit_cpp");
    planning_component_ = config.blackboard->get<moveit_cpp::PlanningComponentPtr>("planning_component");
    planning_group_ = config.blackboard->get<std::string>("planning_group");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    online_ = true;
  }

  void getPoseCallback(const color_pose_msgs::msg::ColorPoseArray& msg)
  {
    if (online_)
    {
      RCLCPP_INFO(node_->get_logger(), "Checking for color pose suggestion");
      final_pose_array = msg;
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pick-z-offset", 0.02, "Offset for picking in z-axis."),
      BT::InputPort<double>("pre-z-offset", 0.12, "Offset for pre in z-axis."),
      BT::InputPort<double>("roll", 0.0, "Roll for gripping."),
      BT::InputPort<double>("pitch", 0.0, "Pitch for gripping."),
      BT::InputPort<double>("yaw", 0.0, "Yaw for gripping."),
      BT::InputPort<std::string>("moving_link", "tool_tip", "The link that is moving."),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[0]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[1]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[2]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[3]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[4]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[5]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[6]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[7]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[8]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[9]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[10]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[11]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[12]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[13]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[14]),
      BT::OutputPort<moveit::core::RobotStatePtr>(color_pose_names[15])

    };
  }

  BT::NodeStatus onStart() override
  {
    RCLCPP_INFO(node_->get_logger(), "Initialising Color Detection.");
    sleep(1.0);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    RCLCPP_INFO(node_->get_logger(), "Initialising Color Detection.");
    geometry_msgs::msg::PoseStamped pre_pose, pick_pose, transformed_pose, final_pose;
    tf2::Quaternion q;
    online_ = false;
    for (int i = 0; i < 16; i++)
    {
      q.setRPY(getInput<double>("roll").value(), getInput<double>("pitch").value(), getInput<double>("yaw").value());
      pre_pose.pose.orientation = tf2::toMsg(q);
      int index = i / 2;
      pre_pose.pose.position.x = final_pose_array.color_poses[index].pose.position.x;
      pre_pose.pose.position.y = final_pose_array.color_poses[index].pose.position.y;

      if (i % 2 == 0)
      {
        pre_pose.pose.position.z = 1.062;
        RCLCPP_INFO(node_->get_logger(), "Even.");
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Odd.");

        pre_pose.pose.position.z = 1.2;
      }

      // RCLCPP_INFO(node_->get_logger(), "Final_Pose: %s", geometry_msgs::msg::to_yaml(final_pose).c_str());
      RCLCPP_INFO(node_->get_logger(), "Pre_Pose: %s", geometry_msgs::msg::to_yaml(pre_pose.pose).c_str());

      auto state = moveit_cpp_->getCurrentState();
      auto jmg2 = state->getJointModelGroup(planning_group_);

      if (!state->setFromIK(jmg2, pre_pose.pose, 10))
      {
        RCLCPP_ERROR(node_->get_logger(), "Could not find IK solution for pre pose.");
        return BT::NodeStatus::FAILURE;
      }

      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      collision_request.contacts = false;
      collision_request.cost = false;
      collision_request.distance = false;

      {
        auto planning_scene_monitor =
            planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_->getPlanningSceneMonitor());
        planning_scene_monitor->checkCollision(collision_request, collision_result, *state);
      }

      if (collision_result.collision)
      {
        RCLCPP_ERROR(node_->get_logger(), "Collision detected for pre pose.");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "color pose names %s", color_pose_names[i]);

      setOutput(color_pose_names[i], state);
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "Halting color detection.");
    error_.store(true);
  }
};

#endif  // DETECT_COLOR_NODE_HPP