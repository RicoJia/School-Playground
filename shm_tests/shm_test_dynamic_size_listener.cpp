// Test 3: Dynamic size messages subscriber
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DynamicSizeListener : public rclcpp::Node
{
public:
  DynamicSizeListener() : Node("dynamic_size_listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "dynamic_topic", 10,
      std::bind(&DynamicSizeListener::topic_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Dynamic size listener started");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received dynamic message: %zu bytes", msg->data.size());
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicSizeListener>());
  rclcpp::shutdown();
  return 0;
}
