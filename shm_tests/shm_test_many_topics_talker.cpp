// Test 2: Many topics publisher (>127 topics to test shared memory limits)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class ManyTopicsTalker : public rclcpp::Node
{
public:
  ManyTopicsTalker() : Node("many_topics_talker"), count_(0)
  {
    // Iceoryx 2.0.5 has a compile-time limit of 512 publishers system-wide.
    // Using 450 to leave headroom for ROS internal publishers (rosout, parameters, etc.)
    const int NUM_TOPICS = 450;
    
    RCLCPP_INFO(this->get_logger(), "Creating %d publishers...", NUM_TOPICS);
    
    // Use BestEffort + KeepLast(1) to minimize SHM usage
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    
    for (int i = 0; i < NUM_TOPICS; i++) {
      std::string topic_name = "test_topic_" + std::to_string(i);
      auto pub = this->create_publisher<std_msgs::msg::Int32>(topic_name, qos);
      publishers_.push_back(pub);
    }
    
    RCLCPP_INFO(this->get_logger(), "Created %zu publishers", publishers_.size());
    
    timer_ = this->create_wall_timer(
      1s, std::bind(&ManyTopicsTalker::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_++;
    
    // Publish to all topics
    for (size_t i = 0; i < publishers_.size(); i++) {
      publishers_[i]->publish(message);
    }
    
    // Throttle logging to reduce overhead
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Published to %zu topics: %d", publishers_.size(), message.data);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> publishers_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManyTopicsTalker>());
  rclcpp::shutdown();
  return 0;
}
