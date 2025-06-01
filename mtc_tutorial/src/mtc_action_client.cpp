#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "action_tutorials_interfaces/srv/planning_action.hpp"

#include <std_msgs/msg/bool.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
/*
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
*/
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

namespace mtc_tutorial
{
  class MtcActionClient : public rclcpp::Node
  {
  public:
    using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
    bool last_goal_succeeded = false;
    std::vector<std::string> actions;
    int i = 0;
    int action_length = actions.size();
    bool action_completed_ = false; // Flag to track if the action is completed
    rclcpp::Client<action_tutorials_interfaces::srv::PlanningAction>::SharedPtr task_planner_service_client;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr action_completed_sub_;

    explicit MtcActionClient(const rclcpp::NodeOptions &options)
        : Node("mtc_action_client", options)
    {
      this->task_planner_service_client = this->create_client<action_tutorials_interfaces::srv::PlanningAction>("task_planner_service");

      // call service to fetch planning action array:
      while (!task_planner_service_client->wait_for_service(1s))
      {
        if (rclcpp::ok())
        {
          RCLCPP_ERROR(
              this->get_logger(),
              "Client interrupted while waiting for service. Terminating...");
          return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Service Unavailable. Waiting for Service...");
      }

      // Add this subscriber to your class
      action_completed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          "/robot_action_completed", 10, // Topic name and QoS depth
          [this](const std_msgs::msg::Bool::SharedPtr msg)
          {
            //action_completed_ = ; // Update the flag based on the message
            if (action_completed_ && msg->data)
            {
              RCLCPP_INFO(this->get_logger(), "Robot has completed the current action.");
              RCLCPP_INFO(this->get_logger(), "Sending next goal.");
              send_goal(this->i);
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "Robot is still executing the current action.");
            }
          });

      auto request = std::make_shared<action_tutorials_interfaces::srv::PlanningAction::Request>();
      // set request variables here, if any
      request->action_name = "hello"; // comment this line if using Empty() message
      // service_done_ = false; // inspired from action client c++ code
      auto result_future = task_planner_service_client->async_send_request(
          request, std::bind(&MtcActionClient::planning_service_response_callback, this,
                             std::placeholders::_1));
      RCLCPP_INFO(
          this->get_logger(),
          "Calling chatgpt service...");
      // ----------------------------------------------------------------

      this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
          this,
          "fibonacci");
    }

    void planning_service_response_callback(
        rclcpp::Client<action_tutorials_interfaces::srv::PlanningAction>::SharedFuture future)
    {
      auto status = future.wait_for(1s);
      if (status == std::future_status::ready)
      {
        // uncomment below line if using Empty() message
        // RCLCPP_INFO(this->get_logger(), "Result: success");
        // comment below line if using Empty() message
        RCLCPP_INFO(this->get_logger(), "Result: success.");
        actions.clear();
        for (const auto &action : future.get()->action_list)
        {
          RCLCPP_INFO(this->get_logger(), "%s, ", action.c_str());
          actions.push_back(action);
        }
        send_goal(this->i);

        // service_done_ = true;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
      }
    }

    void send_goal(int i)
    {
      using namespace std::placeholders;

      // this->timer_->cancel();

      if (!this->client_ptr_->wait_for_action_server())
      {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      auto goal_msg = Fibonacci::Goal();
      goal_msg.action_name = actions[i];

      RCLCPP_INFO(this->get_logger(), "Sending goal");

      auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
      send_goal_options.goal_response_callback =
          std::bind(&MtcActionClient::goal_response_callback, this, _1);
      send_goal_options.feedback_callback =
          std::bind(&MtcActionClient::feedback_callback, this, _1, _2);
      send_goal_options.result_callback =
          std::bind(&MtcActionClient::result_callback, this, _1);
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandleFibonacci::SharedPtr &goal_handle)
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
      std::stringstream ss;
      ss << "Feedback: ";
      ss << feedback->partial_sequence;
      // for (auto number : feedback->partial_sequence) {
      //   ss << number << " ";
      // }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult &result)
    {
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
      {
        std::stringstream ss;
        ss << "Result received: ";
        ss << result.result->sequence;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        this->i += 1;
        if (this->i == this->action_length)
        {
          // rclcpp::shutdown();
          return;
        }
        else
        {
          action_completed_ = true; // Set the flag to true when action is completed
          //send_goal(this->i);
        }

        break;
      }
      case rclcpp_action::ResultCode::ABORTED:
      {
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");

        auto request = std::make_shared<action_tutorials_interfaces::srv::PlanningAction::Request>();
        // set request variables here, if any
        request->action_name = "replan"; // comment this line if using Empty() message
        // service_done_ = false; // inspired from action client c++ code
        auto result_future = task_planner_service_client->async_send_request(
            request, std::bind(&MtcActionClient::planning_service_response_callback, this,
                               std::placeholders::_1));
        RCLCPP_INFO(
            this->get_logger(),
            "Calling chatgpt service...");
        // rclcpp::shutdown();
        break;
      }
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        rclcpp::shutdown();
        return;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        rclcpp::shutdown();
        return;
        break;
      }

    }
  }; // class MtcActionClient

} // namespace mtc_tutorial

RCLCPP_COMPONENTS_REGISTER_NODE(mtc_tutorial::MtcActionClient)