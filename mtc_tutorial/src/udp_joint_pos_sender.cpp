#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>
#include <cstring>

class UdpJointSenderNode : public rclcpp::Node
{
public:

  bool pause_send = false;
  UdpJointSenderNode()
      : Node("udp_joint_sender_node"),
        socket_(io_context_),
        receiver_socket_(io_context_),
        robot_endpoint_(boost::asio::ip::address::from_string("130.251.6.21"), 1700), //130.251.6.21
        receive_endpoint_(boost::asio::ip::udp::v4(), 1701),
        running_(true)
  {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
          latest_joint_positions_ = msg->position;
        });

    task_completed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/task_completed", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          if (msg->data)
          {
            pause_send = false;
            send_joint_positions_udp();
          }
        });

    robot_action_completed_pub_ = this->create_publisher<std_msgs::msg::Bool>("robot_action_completed", 10);

    socket_.open(boost::asio::ip::udp::v4());
    receiver_socket_.open(boost::asio::ip::udp::v4());
    boost::asio::socket_base::receive_buffer_size option(1024 * 1024); // 1 MB
    receiver_socket_.set_option(option);
    receiver_socket_.bind(receive_endpoint_);

    start_receive();
    io_thread_ = std::thread([this]() { io_context_.run(); });
    processing_thread_ = std::thread([this]() { processing_loop(); });
  }

  ~UdpJointSenderNode()
  {
    running_ = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_cv_.notify_all();
    }

    io_context_.stop();
    if (io_thread_.joinable())
      io_thread_.join();
    if (processing_thread_.joinable())
      processing_thread_.join();
  }

private:
  void send_joint_positions_udp()
  {
    pause_send = false;

    if (latest_joint_positions_.size() < 9)
    {
      RCLCPP_WARN(this->get_logger(), "Insufficient joint positions. Expected at least 9.");
      return;
    }

    std::vector<double> sent_joint_positions;
    sent_joint_positions_global.clear();

    for (size_t i = 0; i < latest_joint_positions_.size(); ++i)
    {
      // if (i != 0 && i != 3)
      // {
        sent_joint_positions.push_back(latest_joint_positions_[i]);
        sent_joint_positions_global.push_back(latest_joint_positions_[i]);
      // }
    }

    if (sent_joint_positions.size() != 9)
    {
      RCLCPP_WARN(this->get_logger(), "Unexpected number of joints after filtering. Got %zu", sent_joint_positions.size());
      return;
    }

    std::vector<uint8_t> payload(sizeof(double) * 9);
    std::memcpy(payload.data(), sent_joint_positions.data(), sizeof(double) * 9);
    // print in loop the sent joint positions
    for (size_t i = 0; i < sent_joint_positions.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "Sending joint position %zu: %.6f", i, sent_joint_positions[i]);
    }
    socket_.send_to(boost::asio::buffer(payload), robot_endpoint_);
  }

  void start_receive()
  {
    receiver_socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), receive_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
          RCLCPP_INFO(this->get_logger(), "Received UDP data with size: %zu", bytes_recvd/9);
          if (!ec && bytes_recvd == sizeof(double) * 9)
          {
            std::array<uint8_t, sizeof(double) * 9> data_copy;
            std::memcpy(data_copy.data(), recv_buffer_.data(), sizeof(double) * 9);

            {
              std::lock_guard<std::mutex> lock(queue_mutex_);
              recv_queue_.emplace(std::move(data_copy));
            }
            queue_cv_.notify_one();
          }

          start_receive();
        });
  }

  void processing_loop()
  {

    while (running_)
    {

      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return !recv_queue_.empty() || !running_; });

      if (!running_)
        break;
      RCLCPP_INFO(this->get_logger(), "Received UDP data, processing...");

      auto data = std::move(recv_queue_.front());
      recv_queue_.pop();
      lock.unlock();

      if (pause_send){
      // Clear the queue
      std::queue<std::array<uint8_t, sizeof(double) * 9>> empty;
      std::swap(recv_queue_, empty);
      RCLCPP_WARN(this->get_logger(), "dropping all messages because pause_send is true");

      continue;
      
      }

      double received[9];
      std::memcpy(received, data.data(), sizeof(received));

      const double tolerance = 5e-3;
      bool all_close = true;

      if (sent_joint_positions_global.size() < 9)
      {
        RCLCPP_WARN(this->get_logger(), "No reference joint state to compare against.");
        continue;
      }

      for (size_t i = 0; i < 9; ++i)
      {
        double diff = std::abs(received[i] - sent_joint_positions_global[i]);
        if (diff > tolerance)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Joint %zu mismatch: received=%.6f, expected=%.6f, diff=%.6f",
                      i, received[i], sent_joint_positions_global[i], diff);
          all_close = false;
        }
        else
        {
          RCLCPP_INFO(this->get_logger(),
                      "Joint %zu match: received=%.6f, expected=%.6f, diff=%.6f",
                      i, received[i], sent_joint_positions_global[i], diff);
        }
      }

      if (all_close)
      {

        RCLCPP_INFO(this->get_logger(), "All joint positions match within tolerance.");
        std_msgs::msg::Bool msg;
        msg.data = true;
        robot_action_completed_pub_->publish(msg);
        pause_send = true;

      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Joint positions do not match.");
      }
    }
  }

  // ROS2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_completed_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_action_completed_pub_;
  std::vector<double> latest_joint_positions_;
  std::vector<double> sent_joint_positions_global;

  // UDP
  boost::asio::io_context io_context_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::socket receiver_socket_;
  boost::asio::ip::udp::endpoint robot_endpoint_;
  boost::asio::ip::udp::endpoint receive_endpoint_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::array<uint8_t, sizeof(double) * 9> recv_buffer_;
  std::thread io_thread_;
  std::thread processing_thread_;
  std::atomic<bool> running_;

  // Thread-safe processing queue
  std::queue<std::array<uint8_t, sizeof(double) * 9>> recv_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpJointSenderNode>());
  rclcpp::shutdown();
  return 0;
}

