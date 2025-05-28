#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include <action_tutorials_interfaces/srv/planning_action.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "mtc_tutorial/visibility_control.h"
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
namespace mtc_tutorial
{
class MtcActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  MTC_TUTORIAL_PUBLIC
  explicit MtcActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mtc_action_server", options)
  {
    this->serviceClient_ = this->create_client<action_tutorials_interfaces::srv::PlanningAction>("planning_service");

    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci2",
      std::bind(&MtcActionServer::handle_goal, this, _1, _2),
      std::bind(&MtcActionServer::handle_cancel, this, _1),
      std::bind(&MtcActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  rclcpp::Client<action_tutorials_interfaces::srv::PlanningAction>::SharedPtr serviceClient_;


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request for action: %s", goal->action_name.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MtcActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(50);
    const auto goal = goal_handle->get_goal();
    std::string action_name = goal->action_name;
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    feedback->partial_sequence = "performing action: " + action_name;
    auto result = std::make_shared<Fibonacci::Result>();

      // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->sequence = "goal canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    goal_handle->publish_feedback(feedback);


    // execute the action:

    auto request = std::make_shared<action_tutorials_interfaces::srv::PlanningAction::Request>();
    request->action_name = action_name;

    while (!serviceClient_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
    auto action_execution_result = serviceClient_->async_send_request(request);

    //     // Wait for the result.
    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), action_execution_result) ==
    //     rclcpp::FutureReturnCode::SUCCESS)
    // {
    //   RCLCPP_INFO(this->get_logger(), "Result: %s", action_execution_result.get()->success ? "true" : "false");
    // } else {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to call service string_service");
    // }

    
    loop_rate.sleep();


    //execute the action/call the action function here:
    RCLCPP_INFO(this->get_logger(), "performing action: %s", goal->action_name.c_str());


    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = "action: " + action_name + " complete" ;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "action %s complete", goal->action_name.c_str());
    }
  }
};  // class MtcActionServer

}  // namespace mtc_tutorial

RCLCPP_COMPONENTS_REGISTER_NODE(mtc_tutorial::MtcActionServer)