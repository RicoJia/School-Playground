// Test 2: Many topics subscriber (>127 topics to test shared memory limits)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <vector>
#include <map>
#include <mutex>

class ManyTopicsListener : public rclcpp::Node
{
public:
  ManyTopicsListener(int node_num) : Node("many_topics_listener"), node_num_(node_num)
  {
    // Iceoryx 2.0.5 has IOX_MAX_NUMBER_OF_NOTIFIERS=256 limit (subscribers with callbacks).
    // Using 250 per node to stay safely under the 256 notifier limit with headroom.
    const int NUM_TOPICS = 249;
    
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
      auto sub = this->create_subscription<std_msgs::msg::Int32>(
        topic_name, qos,
        [this, i, topic_id](const std_msgs::msg::Int32::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(metrics_mutex_);
          receive_counts_[i]++;
          // Log every 100 to reduce console spam
          if (receive_counts_[i] % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Topic %d received %d messages", topic_id, receive_counts_[i]);
          }
        },
        sub_options);
      subscriptions_.push_back(sub);
    }
    
    RCLCPP_INFO(this->get_logger(), "Created %zu subscriptions", subscriptions_.size());
  }

private:
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> subscriptions_;
  std::map<int, int> receive_counts_;
  std::mutex metrics_mutex_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
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
  
  // Use MultiThreadedExecutor to process messages from multiple topics concurrently,
  // preventing the "bursty" behavior caused by sequential processing
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ManyTopicsListener>(node_num);
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
