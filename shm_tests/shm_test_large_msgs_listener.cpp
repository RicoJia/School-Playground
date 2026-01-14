// Test 1: Large message subscriber (should use shared memory with pre-allocation)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LargeMsgListener : public rclcpp::Node
{
public:
  LargeMsgListener() : Node("large_msg_listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "large_topic", 10,
      std::bind(&LargeMsgListener::topic_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Large message listener started");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received large message: %zu bytes", msg->data.size());
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LargeMsgListener>());
  rclcpp::shutdown();
  return 0;
}
