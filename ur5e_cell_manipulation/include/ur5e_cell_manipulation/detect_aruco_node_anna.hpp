#ifndef DETECT_ARUCO_NODE_HPP
#define DETECT_ARUCO_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <aruco_msgs/msg/marker_array.hpp>
#include <atomic>

class DetectArucoNode : public BT::StatefulActionNode
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr sub_markers_;
  std::atomic_bool online_;
  std::atomic_int marker_id_;
  std::atomic_int counter_;
  std::atomic_int required_poses_;
  std::atomic_bool error_;
  std::vector<geometry_msgs::msg::PoseStamped> res_markers_;
  rclcpp::TimerBase::SharedPtr timer_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  std::string planning_group_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

public:
  DetectArucoNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Initialising Execute Node.");
    sub_markers_ = node_->create_subscription<aruco_msgs::msg::MarkerArray>(
        "marker_publisher/markers", 5,
        std::bind(&DetectArucoNode::getMarkerCallback, this, std::placeholders::_1));
    online_.store(false);
    moveit_cpp_ = config.blackboard->get<moveit_cpp::MoveItCppPtr>("moveit_cpp");
    planning_component_ =
        config.blackboard->get<moveit_cpp::PlanningComponentPtr>("planning_component");
    planning_group_ = config.blackboard->get<std::string>("planning_group");
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("~/marker", 10);
  }

  void getMarkerCallback(const aruco_msgs::msg::MarkerArray& msg)
  {
    if (online_.load())
    {
      int marker_id = marker_id_.load();
      RCLCPP_INFO(node_->get_logger(), "Checking for marker ID: %d", marker_id);

      auto func = [marker_id](const aruco_msgs::msg::Marker& x) { return (int)x.id == marker_id; };
      //RCLCPP_INFO(node_->get_logger(), "MarkerArray: %s", aruco_msgs::msg::to_yaml(msg).c_str());
      // Find marker with our ID
      auto res = std::find_if(msg.markers.begin(), msg.markers.end(), std::move(func));
      
      // If marker with our ID is found, store it for later processing.
      if (res != msg.markers.end())
      {
        
        geometry_msgs::msg::PoseStamped marker_posestamped;
        marker_posestamped.header = res->header;
        marker_posestamped.pose = res->pose.pose;
        res_markers_.push_back(marker_posestamped);
        counter_++;
        RCLCPP_INFO(node_->get_logger(), "Counter: %d", counter_.load());
      }
      if (required_poses_.load() == counter_.load())
      {
        online_.store(false);
        timer_->cancel();
      }
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("marker_id"),
      BT::InputPort<int>("required_poses", 5, "Poses required for success."),
      BT::InputPort<int>("timeout", 1000, "Timeout in milliseconds."),
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
    RCLCPP_INFO(node_->get_logger(), "Initialising Aruco Detection.");
    if (!getInput<int>("marker_id").has_value())
    {
      RCLCPP_ERROR(node_->get_logger(), "No marker id provided");
      return BT::NodeStatus::FAILURE;
    }
    res_markers_.clear();
    marker_id_.store(getInput<int>("marker_id").value());
    required_poses_.store(getInput<int>("required_poses").value());
    int timeout = getInput<int>("timeout").value();

    online_.store(true);
    counter_.store(0);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(timeout), [this]() {
      if (online_.load())
      {
        RCLCPP_INFO(node_->get_logger(), "Aruco Detection timed out.");
        online_.store(false);
        error_.store(true);
      }
      timer_->cancel();
    });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!online_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "Aruco Detection done.");
      if (error_.load())
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Timeout reached and insufficient marker poses returned.");
        return BT::NodeStatus::FAILURE;
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Markers found");
        // make sure all poses are in the world frame
        for (auto& pose : res_markers_)
        {
          if (pose.header.frame_id != "world")
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "Marker pose not in world frame, change in aruco_ros launch file.");
            return BT::NodeStatus::FAILURE;
          }
        }
        // create average position
        geometry_msgs::msg::PoseStamped average_pose = std::accumulate(
            res_markers_.begin(), res_markers_.end(), geometry_msgs::msg::PoseStamped(),
            [](geometry_msgs::msg::PoseStamped& a, geometry_msgs::msg::PoseStamped& b) {
              a.pose.position.x += b.pose.position.x;
              a.pose.position.y += b.pose.position.y;
              a.pose.position.z += b.pose.position.z;
              return a;
            });
        average_pose.pose.position.x /= res_markers_.size();
        average_pose.pose.position.y /= res_markers_.size();
        average_pose.pose.position.z /= res_markers_.size();

        // fill
        geometry_msgs::msg::PoseStamped pre_pose;
        geometry_msgs::msg::PoseStamped pick_pose;
        tf2::Quaternion q;
        q.setRPY(getInput<double>("roll").value(), getInput<double>("pitch").value(),
                 getInput<double>("yaw").value());
        pre_pose.pose.orientation = tf2::toMsg(q);
        pick_pose.pose.orientation = tf2::toMsg(q);


         geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::TransformStamped t_sum;


    for (int i = 0; i < required_poses_.load(); i++)
    {
      try
      {
        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, tf2::durationFromSec(0.5));
        t_sum.transform.translation.x =  t_sum.transform.translation.x +t.transform.translation.x;
        t_sum.transform.translation.y =  t_sum.transform.translation.x +t.transform.translation.y;
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_INFO(node_->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(),
                    ex.what());
      }
      
    }
    t_sum.transform.translation.x =  t_sum.transform.translation.x/required_poses_.load(); 
    t_sum.transform.translation.y =  t_sum.transform.translation.y/required_poses_.load(); 


    

        pre_pose.pose.position.x = average_pose.pose.position.x;
        pre_pose.pose.position.y = average_pose.pose.position.y;
        pre_pose.pose.position.z =
            average_pose.pose.position.z + getInput<double>("pre-z-offset").value();

        pick_pose.pose.position.x = average_pose.pose.position.x;
        pick_pose.pose.position.y = average_pose.pose.position.y;
        pick_pose.pose.position.z =
            average_pose.pose.position.z + getInput<double>("pick-z-offset").value();

        RCLCPP_INFO(node_->get_logger(), "Pick_Pose: %s", geometry_msgs::msg::to_yaml(pick_pose).c_str());
        RCLCPP_INFO(node_->get_logger(), "Pre_Pose: %s", geometry_msgs::msg::to_yaml(pre_pose).c_str());

        visualization_msgs::msg::Marker pick_pose_marker;
        pick_pose_marker.header.frame_id="world";
        pick_pose_marker.id=20000;
        pick_pose_marker.type=visualization_msgs::msg::Marker::CUBE;
        pick_pose_marker.action=visualization_msgs::msg::Marker::ADD;
        pick_pose_marker.pose = pick_pose.pose;
        pick_pose_marker.scale.x = 1.0;
        pick_pose_marker.scale.y = 1.0;
        pick_pose_marker.scale.z = 1.0;
        pick_pose_marker.color.r = 1.0;

        marker_publisher_->publish(pick_pose_marker);

        auto pick_state = moveit_cpp_->getCurrentState();
        auto pre_state = moveit_cpp_->getCurrentState();
        auto jmg1 = pick_state->getJointModelGroup(planning_group_);
        auto jmg2 = pre_state->getJointModelGroup(planning_group_);

        if (!pick_state->setFromIK(jmg1, pick_pose.pose, 5))
        {
          RCLCPP_ERROR(node_->get_logger(), "Could not find IK solution for pick pose.");
          return BT::NodeStatus::FAILURE;
        }
        if (!pre_state->setFromIK(jmg2, pre_pose.pose, 5))
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
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "Halting aruco marker detection.");
    online_.store(false);
    error_.store(true);
    timer_->cancel();
  }
};

#endif  // DETECT_ARUCO_NODE_HPP
