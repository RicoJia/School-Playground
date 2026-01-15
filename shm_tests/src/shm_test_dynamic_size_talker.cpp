// Test 3: Dynamic size messages (tests dynamic memory allocation vs pre-allocation)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class DynamicSizeTalker : public rclcpp::Node
{
public:
  DynamicSizeTalker() : Node("dynamic_size_talker"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("dynamic_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DynamicSizeTalker::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Dynamic size talker started");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    
    // Randomly vary message size from 100 bytes to 100KB
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(100, 102400);
    
    int size = dis(gen);
    std::string data(size, 'X');
    message.data = "Msg " + std::to_string(count_++) + " [" + std::to_string(size) + "B]: " + data;
    
    RCLCPP_INFO(this->get_logger(), "Publishing dynamic message: %zu bytes", message.data.size());
    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicSizeTalker>());
  rclcpp::shutdown();
  return 0;
}
