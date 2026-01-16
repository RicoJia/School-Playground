// Test 2: Many topics subscriber (>127 topics to test shared memory limits)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include <deque>

class ManyTopicsListener : public rclcpp::Node
{
public:
  ManyTopicsListener(int node_num) 
    : Node("many_topics_listener"), 
      node_num_(node_num), 
      total_latency_us_(0), 
      total_messages_(0),
      last_report_time_(std::chrono::steady_clock::now())
  {
    // Declare ROS2 parameters
    this->declare_parameter<int>("num_topics", 498);
    this->declare_parameter<int>("message_length", 0);
    
    // Get parameter values
    num_topics_ = this->get_parameter("num_topics").as_int();
    message_length_ = this->get_parameter("message_length").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Node %d: Creating %d subscriptions with expected %d byte messages...", 
                node_num_, num_topics_, message_length_ + 8);
    
    // Use BestEffort + KeepLast(1) to prevent queue buildup
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    
    // Reentrant callback group allows parallel processing with MultiThreadedExecutor
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_;
    
    for (int i = 0; i < num_topics_; i++) {
      int topic_id = node_num_ * num_topics_ + i;
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
          }
        },
        sub_options);
      subscriptions_.push_back(sub);
    }
    
    RCLCPP_INFO(this->get_logger(), "Created %zu subscriptions", subscriptions_.size());
    
    // Create timer for periodic statistics reporting
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ManyTopicsListener::report_statistics, this));
  }

private:
  void report_statistics()
  {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    if (total_messages_ == 0) {
      RCLCPP_INFO(this->get_logger(), "No messages received yet");
      return;
    }
    
    // Manage deque size - keep only last 1000 latencies
    const size_t MAX_LATENCIES = 1000;
    if (latencies_.size() > MAX_LATENCIES) {
      latencies_.erase(latencies_.begin(), latencies_.begin() + (latencies_.size() - MAX_LATENCIES));
    }
    
    // Calculate statistics
    double avg_latency_us = total_latency_us_ / (double)total_messages_;
    
    // Calculate rolling average from recent latencies
    double rolling_avg_us = 0.0;
    double min_latency_us = 0.0;
    double max_latency_us = 0.0;
    double median_latency_us = 0.0;
    
    if (!latencies_.empty()) {
      rolling_avg_us = std::accumulate(latencies_.begin(), latencies_.end(), 0.0) / latencies_.size();
      
      auto sorted_latencies = std::vector<int64_t>(latencies_.begin(), latencies_.end());
      std::sort(sorted_latencies.begin(), sorted_latencies.end());
      
      min_latency_us = sorted_latencies.front();
      max_latency_us = sorted_latencies.back();
      median_latency_us = sorted_latencies[sorted_latencies.size() / 2];
    }
    
    // Count active subscriptions (those that have received at least one message)
    int active_subs = 0;
    for (const auto& pair : receive_counts_) {
      if (pair.second > 0) active_subs++;
    }
    
    RCLCPP_INFO(this->get_logger(),
                "Stats: total_msgs=%ld, active_topics=%d/%d | "
                "Latency: avg=%.1f µs, rolling_avg=%.1f µs, min=%.1f µs, max=%.1f µs, median=%.1f µs",
                total_messages_, active_subs, num_topics_,
                avg_latency_us, rolling_avg_us, min_latency_us, max_latency_us, median_latency_us);
    
    last_report_time_ = std::chrono::steady_clock::now();
  }
  
  std::vector<rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr> subscriptions_;
  std::map<int, int> receive_counts_;
  std::mutex metrics_mutex_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  int node_num_;
  int num_topics_;
  int message_length_;
  int64_t total_latency_us_;
  int64_t total_messages_;
  std::deque<int64_t> latencies_;
  std::chrono::steady_clock::time_point last_report_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <node_num> [--ros-args --param name:=value ...]" << std::endl;
    std::cerr << "  node_num: integer identifier for this node (e.g., 0, 1, 2...)" << std::endl;
    std::cerr << "  ROS2 Parameters:" << std::endl;
    std::cerr << "    --param num_topics:=N (default: 498)" << std::endl;
    std::cerr << "    --param message_length:=N (default: 0)" << std::endl;
    std::cerr << "Example: " << argv[0] << " 0 --ros-args --param num_topics:=100 --param message_length:=1024" << std::endl;
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
