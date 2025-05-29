#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <boost/asio.hpp>

#include <vector>
#include <cstdint>
#include <cstring>

class UdpJointSenderNode : public rclcpp::Node {
public:
  UdpJointSenderNode()
      : Node("udp_joint_sender_node"),
        socket_(io_context_),
        robot_endpoint_(boost::asio::ip::address::from_string("192.168.1.100"), 5005) {  // Change IP and port
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          latest_joint_positions_ = msg->position;
        });

    task_completed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/task_completed", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            send_joint_positions_udp();
          }
        });

    socket_.open(boost::asio::ip::udp::v4());
  }

private:
  void send_joint_positions_udp() {
    uint32_t size = latest_joint_positions_.size();
    std::vector<uint8_t> payload(sizeof(uint32_t) + sizeof(double) * size);

    // Copy size (first 4 bytes)
    std::memcpy(payload.data(), &size, sizeof(uint32_t));

    // Copy double values
    std::memcpy(payload.data() + sizeof(uint32_t),
                latest_joint_positions_.data(),
                sizeof(double) * size);

    // Send over UDP
    socket_.send_to(boost::asio::buffer(payload), robot_endpoint_);
    RCLCPP_INFO(this->get_logger(), "Sent %u joint positions over UDP", size);
    
    for (size_t i = 0; i < 9; ++i) 
    {
      RCLCPP_INFO(this->get_logger(), "Joint %zu: %f", i, latest_joint_positions_[i]);
    }
  }

  // ROS2 Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_completed_sub_;

  // Last joint positions received
  std::vector<double> latest_joint_positions_;

  // Boost.Asio UDP
  boost::asio::io_context io_context_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint robot_endpoint_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<UdpJointSenderNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
