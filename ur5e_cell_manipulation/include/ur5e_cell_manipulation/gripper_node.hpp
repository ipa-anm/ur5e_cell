#ifndef GRIPPER_NODE_HPP
#define GRIPPER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <atomic>

class GripperActionNode : public BT::StatefulActionNode
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic_bool command_send_pin_0_;
  std::atomic_bool command_send_pin_1_;
  std::atomic_bool error_;

public:
  GripperActionNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Initialising Gripper Node.");
    client_ = node_->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
    command_send_pin_0_ = false;
    command_send_pin_1_ = false;
    error_.store(false);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action"),
      BT::InputPort<int>("pin_0"),
      BT::InputPort<int>("pin_1"),
      BT::InputPort<int>("timeout_ms", 1000, "The timeout, default is 100ms."),
      BT::InputPort<bool>("fake", true, "If this is only faked."),
    };
  }

  BT::NodeStatus onStart() override
  {
    RCLCPP_INFO(node_->get_logger(), "Initialising Gripper Node.");
    if (getInput<std::string>("action").has_value() == false)
    {
      RCLCPP_ERROR(node_->get_logger(), "The action input is not set.");
      return BT::NodeStatus::FAILURE;
    }
    if (getInput<int>("pin_0").has_value() == false)
    {
      RCLCPP_ERROR(node_->get_logger(), "The pin_0 input is not set.");
      return BT::NodeStatus::FAILURE;
    }
    if (getInput<int>("pin_1").has_value() == false)
    {
      RCLCPP_ERROR(node_->get_logger(), "The pin_1 input is not set.");
      return BT::NodeStatus::FAILURE;
    }
    if (getInput<int>("timeout_ms").has_value() == false)
    {
      RCLCPP_ERROR(node_->get_logger(), "The timeout_ms input is not set.");
      return BT::NodeStatus::FAILURE;
    }
    std::string action = getInput<std::string>("action").value();
    int pin_0 = getInput<int>("pin_0").value();
    int pin_1 = getInput<int>("pin_1").value();
    int timeout_ms = getInput<int>("timeout_ms").value();
    bool fake = getInput<bool>("fake").value();
    if (fake)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "Faking %s gripper on pin_0:= %hhi, pin_1:=%hhi with timeout: %hi ms",
                  action.c_str(), pin_0, pin_1, timeout_ms);
      return BT::NodeStatus::SUCCESS;
    }
    error_.store(false);
    RCLCPP_INFO(node_->get_logger(),
                "Start %s gripper on pin_0:= %hhi, pin_1:=%hhi with timeout: %hi ms",
                action.c_str(), pin_0, pin_1, timeout_ms);

    if (!client_->wait_for_service(std::chrono::milliseconds(10)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Service not available.");
      return BT::NodeStatus::FAILURE;
    }

    auto command_0_callback =
        [this](rclcpp::Client<ur_msgs::srv::SetIO>::SharedFutureWithRequest future) {
          auto result = future.get();
          if (result.second->success)
          {
            command_send_pin_0_.store(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Gripper command failed on pin_0.");
            error_.store(false);
          }
        };

    auto command_1_callback =
        [this](rclcpp::Client<ur_msgs::srv::SetIO>::SharedFutureWithRequest future) {
          auto result = future.get();
          if (result.second->success)
          {
            command_send_pin_1_.store(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Gripper command failed on pin_0.");
            error_.store(false);
          }
        };

    auto request_pin_0 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    auto request_pin_1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request_pin_0->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    request_pin_1->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    request_pin_0->pin = pin_0;
    request_pin_1->pin = pin_1;

    if (action == "open")
    {
      request_pin_0->state = ur_msgs::srv::SetIO::Request::STATE_OFF;
      request_pin_1->state = ur_msgs::srv::SetIO::Request::STATE_ON;
    }
    else if (action == "close")
    {
      request_pin_0->state = ur_msgs::srv::SetIO::Request::STATE_ON;
      request_pin_1->state = ur_msgs::srv::SetIO::Request::STATE_OFF;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unknown action: %s", action.c_str());
      return BT::NodeStatus::FAILURE;
    }

    this->client_->async_send_request(request_pin_0, command_0_callback);
    this->client_->async_send_request(request_pin_1, command_1_callback);
    timer_ = this->node_->create_wall_timer(std::chrono::milliseconds(timeout_ms), [this]() {
      RCLCPP_ERROR(node_->get_logger(), "Gripper command timed out.");
      error_.store(true);
      timer_->cancel();
    });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (command_send_pin_0_ && command_send_pin_1_)
    {
      RCLCPP_INFO(node_->get_logger(), "Gripper command executed.");
      timer_->cancel();
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      return BT::NodeStatus::SUCCESS;
    }
    if (error_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Gripper command failed.");
      RCLCPP_ERROR(node_->get_logger(),
                   "Gripper command failed, removed %ld pending client requests",
                   client_->prune_pending_requests());
      timer_->cancel();
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "Halting gripper action.");
    timer_->cancel();
    client_->prune_pending_requests();
  }
};

#endif  // GRIPPER_NODE_HPP
