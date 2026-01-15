// Test 1: Large message publisher (should use shared memory with pre-allocation)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

class LargeMsgTalker : public rclcpp::Node
{
public:
  LargeMsgTalker() : Node("large_msg_talker"), count_(0)
  {
    // Create large message publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("large_topic", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&LargeMsgTalker::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Large message talker started");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    // Create a large message (10KB)
    std::string large_data(10240, 'A');
    message.data = "Message " + std::to_string(count_++) + ": " + large_data;
    
    RCLCPP_INFO(this->get_logger(), "Publishing large message: %zu bytes", message.data.size());
    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LargeMsgTalker>());
  rclcpp::shutdown();
  return 0;
}
