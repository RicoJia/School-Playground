// Test 2: Many topics publisher (>127 topics to test shared memory limits)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class ManyTopicsTalker : public rclcpp::Node
{
public:
  ManyTopicsTalker(int node_num) : Node("many_topics_talker"), count_(0), node_num_(node_num)
  {
    // Iceoryx 2.0.5 has IOX_MAX_NUMBER_OF_NOTIFIERS=256 limit.
    // Using 250 per node to stay safely under the 256 notifier limit with headroom for ROS internals.
    const int NUM_TOPICS = 250;
    
    RCLCPP_INFO(this->get_logger(), "Node %d: Creating %d publishers...", node_num_, NUM_TOPICS);
    
    // Use BestEffort + KeepLast(1) to minimize SHM usage
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    
    for (int i = 0; i < NUM_TOPICS; i++) {
      int topic_id = node_num_ * NUM_TOPICS + i;
      std::string topic_name = "test_topic_" + std::to_string(topic_id);
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
  int node_num_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <node_num>" << std::endl;
    std::cerr << "  node_num: integer identifier for this node (e.g., 0, 1, 2...)" << std::endl;
    return 1;
  }
  
  int node_num = std::atoi(argv[1]);
  if (node_num < 0) {
    std::cerr << "Error: node_num must be non-negative" << std::endl;
    return 1;
  }
  
  rclcpp::spin(std::make_shared<ManyTopicsTalker>(node_num));
  rclcpp::shutdown();
  return 0;
}
