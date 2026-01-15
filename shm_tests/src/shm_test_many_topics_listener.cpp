// Test 2: Many topics subscriber (>127 topics to test shared memory limits)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include <vector>
#include <map>
#include <mutex>
#include <chrono>
#include <numeric>
#include <cstring>

class ManyTopicsListener : public rclcpp::Node
{
public:
  ManyTopicsListener(int node_num) : Node("many_topics_listener"), node_num_(node_num), total_latency_us_(0), total_messages_(0)
  {
    // Creating 498 subscriptions
    const int NUM_TOPICS = 498;
    
    RCLCPP_INFO(this->get_logger(), "Node %d: Creating %d subscriptions...", node_num_, NUM_TOPICS);
    
    // Use BestEffort + KeepLast(1) to prevent queue buildup
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    
    // Reentrant callback group allows parallel processing with MultiThreadedExecutor
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_;
    
    for (int i = 0; i < NUM_TOPICS; i++) {
      int topic_id = node_num_ * NUM_TOPICS + i;
      std::string topic_name = "test_topic_" + std::to_string(topic_id);
      auto sub = this->create_subscription<std_msgs::msg::ByteMultiArray>(
        topic_name, qos,
        [this, i, topic_id](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
          auto receive_time = std::chrono::high_resolution_clock::now();
          
          std::lock_guard<std::mutex> lock(metrics_mutex_);
          receive_counts_[i]++;
          
          // Extract timestamp from message
          if (msg->data.size() >= 12) {
            int64_t send_timestamp_ns;
            int32_t msg_count;
            std::memcpy(&send_timestamp_ns, &msg->data[0], sizeof(send_timestamp_ns));
            std::memcpy(&msg_count, &msg->data[8], sizeof(msg_count));
            
            auto receive_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              receive_time.time_since_epoch()).count();
            
            int64_t latency_ns = receive_timestamp_ns - send_timestamp_ns;
            int64_t latency_us = latency_ns / 1000;
            
            total_latency_us_ += latency_us;
            total_messages_++;
            latencies_.push_back(latency_us);
            
            // Keep only last 1000 latencies to compute rolling average
            if (latencies_.size() > 1000) {
              latencies_.erase(latencies_.begin());
            }
            
            // Log every 10 messages to reduce console spam
            if (receive_counts_[i] % 10 == 0) {
              double avg_latency = total_latency_us_ / (double)total_messages_;
              double rolling_avg = std::accumulate(latencies_.begin(), latencies_.end(), 0.0) / latencies_.size();
              RCLCPP_INFO(this->get_logger(), 
                         "Topic %d: msg %d (100B), latency=%ld µs, avg=%.1f µs, rolling_avg=%.1f µs",
                         topic_id, msg_count, latency_us, avg_latency, rolling_avg);
            }
          }
        },
        sub_options);
      subscriptions_.push_back(sub);
    }
    
    RCLCPP_INFO(this->get_logger(), "Created %zu subscriptions", subscriptions_.size());
  }

private:
  std::vector<rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr> subscriptions_;
  std::map<int, int> receive_counts_;
  std::mutex metrics_mutex_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  int node_num_;
  int64_t total_latency_us_;
  int64_t total_messages_;
  std::vector<int64_t> latencies_;
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
  
  // Use MultiThreadedExecutor to process messages from multiple topics concurrently,
  // preventing the "bursty" behavior caused by sequential processing
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ManyTopicsListener>(node_num);
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
