// Test 2: Many topics publisher (>127 topics to test shared memory limits)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include <chrono>
#include <vector>
#include <cstring>

using namespace std::chrono_literals;

class ManyTopicsTalker : public rclcpp::Node
{
public:
  ManyTopicsTalker(int node_num) : Node("many_topics_talker"), count_(0), node_num_(node_num)
  {
    // Creating 498 publishers
    const int NUM_TOPICS = 498;
    
    RCLCPP_INFO(this->get_logger(), "Node %d: Creating %d publishers...", node_num_, NUM_TOPICS);
    
    // Use BestEffort + KeepLast(1) to minimize SHM usage
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    
    for (int i = 0; i < NUM_TOPICS; i++) {
      int topic_id = node_num_ * NUM_TOPICS + i;
      std::string topic_name = "test_topic_" + std::to_string(topic_id);
      auto pub = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic_name, qos);
      publishers_.push_back(pub);
    }
    
    RCLCPP_INFO(this->get_logger(), "Created %zu publishers", publishers_.size());
    
    timer_ = this->create_wall_timer(
      1s, std::bind(&ManyTopicsTalker::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Create message with timestamp + counter (8 bytes + 4 bytes = 12 bytes)
    auto message = std_msgs::msg::ByteMultiArray();
    message.data.resize(12000);

    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Get current timestamp in nanoseconds
    auto send_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      start_time.time_since_epoch()).count();
    
    std::memcpy(&message.data[0], &send_timestamp_ns, sizeof(send_timestamp_ns));
    std::memcpy(&message.data[8], &count_, sizeof(count_));
    
    // Publish to all topics
    for (size_t i = 0; i < publishers_.size(); i++) {
      publishers_[i]->publish(message);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto publish_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time).count();
    
    // Calculate average time per publish
    double avg_time_per_topic_us = publish_duration_us / (double)publishers_.size();
    
    // Throttle logging to reduce overhead
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Published to %zu topics: msg=%d, total_time=%ld µs, avg_per_topic=%.2f µs",
                         publishers_.size(), count_, publish_duration_us, avg_time_per_topic_us);
    
    count_++;
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr> publishers_;
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
