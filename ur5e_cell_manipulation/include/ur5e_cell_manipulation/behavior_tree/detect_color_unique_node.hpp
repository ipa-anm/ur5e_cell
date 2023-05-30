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
#include <color_pose_msgs/msg/color_pose.hpp>
#include <atomic>
#include <iostream>
#include <fstream>


class DetectColorNode : public BT::StatefulActionNode
{
protected:
  std::atomic_int required_poses_ = 3;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  // color_pose_msgs::msg::ColorPose final_pose_;
  std::atomic_int counter_;
  std::atomic_bool error_;
  rclcpp::TimerBase::SharedPtr timer_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  std::string planning_group_;

public:
  DetectColorNode(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Initialising Execute Node.");
    sleep(1.0);

    // pose_ = node_->create_subscription<color_pose_msgs::msg::ColorPose>(
    //     "color_pose_estimation/color_pose", 30,
    //     std::bind(&DetectColorNode::getPoseCallback, this, std::placeholders::_1));
    moveit_cpp_ = config.blackboard->get<moveit_cpp::MoveItCppPtr>("moveit_cpp");
    planning_component_ = config.blackboard->get<moveit_cpp::PlanningComponentPtr>("planning_component");
    planning_group_ = config.blackboard->get<std::string>("planning_group");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

  // void getPoseCallback(const color_pose_msgs::msg::ColorPose& msg)
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Checking for color pose suggestion");

  //   final_pose_ = msg;
  //   timer_->cancel();
  // }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pick-z-offset", 0.02, "Offset for picking in z-axis."),
      BT::InputPort<double>("pre-z-offset", 0.12, "Offset for pre in z-axis."),
      BT::InputPort<double>("roll", 0.0, "Roll for gripping."),
      BT::InputPort<double>("pitch", 0.0, "Pitch for gripping."),
      BT::InputPort<double>("yaw", 0.0, "Yaw for gripping."),
      BT::InputPort<std::string>("moving_link", "tool_tip", "The link that is moving."),
      BT::OutputPort<moveit::core::RobotStatePtr>("pick_robot_state"),
      BT::OutputPort<moveit::core::RobotStatePtr>("pre_robot_state"),
    };
  }

  BT::NodeStatus onStart() override
  {
    RCLCPP_INFO(node_->get_logger(), "Initialising Color Detection.");

    return BT::NodeStatus::RUNNING;

  }



  BT::NodeStatus onRunning() override
  {
    RCLCPP_INFO(node_->get_logger(), "Initialising Color Detection.");

    geometry_msgs::msg::PoseStamped pre_pose, pick_pose, transformed_pose, final_pose;
    tf2::Quaternion q;

    transformed_pose.pose.position.x = 0;
    transformed_pose.pose.position.y = 0;
    transformed_pose.pose.position.z = 0;

    transformed_pose.pose.orientation.x = 0;
    transformed_pose.pose.orientation.y = 0;
    transformed_pose.pose.orientation.z = 0;
    transformed_pose.pose.orientation.w = 0;
    std::string toFrameRel = "world";
    std::string fromFrameRel = "box_frame";

    geometry_msgs::msg::TransformStamped t, t_sum;


    t.transform.translation.x = 0; 
    t.transform.translation.y = 0; 
    t.transform.translation.z = 0; 
    t_sum.transform.translation.x = 0; 
    t_sum.transform.translation.y = 0; 
    t_sum.transform.translation.z = 0; 

    RCLCPP_INFO(node_->get_logger(), "Created_transformation.");

    for (int i = 0; i < required_poses_; i++)
    {
      try
      {
        rclcpp::sleep_for(std::chrono::milliseconds(700));

        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, tf2::durationFromSec(1));

        RCLCPP_INFO(node_->get_logger(), "Counter: %d", i);

        t_sum.transform.translation.x = t_sum.transform.translation.x + t.transform.translation.x;
        t_sum.transform.translation.y = t_sum.transform.translation.y + t.transform.translation.y;
        t_sum.transform.translation.z = t_sum.transform.translation.z + t.transform.translation.z;
      }
      catch (tf2::TransformException& ex)
      {
        RCLCPP_INFO(node_->get_logger(), "Exception: %s", ex.what());
        i = i-1; 
      }
      RCLCPP_INFO(node_->get_logger(), "Sum_Transformation: %s", geometry_msgs::msg::to_yaml(t_sum.transform).c_str());

    }

    t_sum.transform.translation.x = t_sum.transform.translation.x / required_poses_.load();
    t_sum.transform.translation.y = t_sum.transform.translation.y / required_poses_.load();
    t_sum.transform.translation.z = t_sum.transform.translation.z / required_poses_.load();
    std::ofstream file("plot.csv", std::ios::app);    
    std::string newLine = std::to_string(t_sum.transform.translation.x) + "," + std::to_string(t_sum.transform.translation.y) + "," + std::to_string(t_sum.transform.translation.z);
    file << newLine<<std::endl;
    file.close();

    RCLCPP_INFO(node_->get_logger(), "Sum_Transformation_AFTER: %s", geometry_msgs::msg::to_yaml(t_sum.transform).c_str());

    tf2::doTransform(transformed_pose, final_pose, t_sum);

    q.setRPY(getInput<double>("roll").value(), getInput<double>("pitch").value(), getInput<double>("yaw").value());
    pre_pose.pose.orientation = tf2::toMsg(q);
    pick_pose.pose.orientation = tf2::toMsg(q);

    pre_pose.pose.position.x = final_pose.pose.position.x;
    pre_pose.pose.position.y = final_pose.pose.position.y;
    pre_pose.pose.position.z = 1.15; 

    pick_pose.pose.position.x = final_pose.pose.position.x;
    pick_pose.pose.position.y = final_pose.pose.position.y;
    pick_pose.pose.position.z = 1.062;
    // RCLCPP_INFO(node_->get_logger(), "Final_Pose: %s", geometry_msgs::msg::to_yaml(final_pose).c_str());
    RCLCPP_INFO(node_->get_logger(), "Pre_Pose: %s", geometry_msgs::msg::to_yaml(pre_pose.pose).c_str());

    auto pick_state = moveit_cpp_->getCurrentState();
    auto pre_state = moveit_cpp_->getCurrentState();
    auto jmg1 = pick_state->getJointModelGroup(planning_group_);
    auto jmg2 = pre_state->getJointModelGroup(planning_group_);


    if (!pick_state->setFromIK(jmg1, pick_pose.pose, 10))
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not find IK solution for pick pose.");
      return BT::NodeStatus::FAILURE;
    }
    if (!pre_state->setFromIK(jmg2, pre_pose.pose, 10))
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
      planning_scene_monitor->checkCollision(collision_request, collision_result, *pick_state);
    }

    if (collision_result.collision)
    {
      RCLCPP_ERROR(node_->get_logger(), "Collision detected for pick pose.");
      return BT::NodeStatus::FAILURE;
    }

    {
      auto planning_scene_monitor =
          planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_->getPlanningSceneMonitor());
      planning_scene_monitor->checkCollision(collision_request, collision_result, *pre_state);
    }

    if (collision_result.collision)
    {
      RCLCPP_ERROR(node_->get_logger(), "Collision detected for pre pose.");
      return BT::NodeStatus::FAILURE;
    }

    setOutput("pick_robot_state", pick_state);
    setOutput("pre_robot_state", pre_state);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "Halting color detection.");
    error_.store(true);
    timer_->cancel();
  }
};

#endif  // DETECT_COLOR_NODE_HPP