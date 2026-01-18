// Test 2: Many topics publisher with fixed-size messages
#include "rclcpp/rclcpp.hpp"
#include "shm_tests/msg/fixed_data1_kb.hpp"
#include "shm_tests/msg/fixed_data15_kb.hpp"
#include <chrono>
#include <vector>
#include <cstring>
#include <string>

using namespace std::chrono_literals;

class ManyTopicsTalker : public rclcpp::Node
{
public:
  ManyTopicsTalker(int node_num) : Node("many_topics_talker"), count_(0), node_num_(node_num)
  {
    // Declare ROS2 parameters
    this->declare_parameter<int>("num_topics", 10);
    this->declare_parameter<std::string>("message_size", "1KB");  // "1KB" or "15KB"
    this->declare_parameter<bool>("zero_copy", true);
    
    // Get parameter values
    num_topics_ = this->get_parameter("num_topics").as_int();
    message_size_ = this->get_parameter("message_size").as_string();
    zero_copy_ = this->get_parameter("zero_copy").as_bool();
    
    std::string mode = zero_copy_ ? "zero-copy" : "standard";
    
    RCLCPP_INFO(this->get_logger(), "Node %d: Creating %d publishers with %s messages (%s mode)...", 
                node_num_, num_topics_, message_size_.c_str(), mode.c_str());
    
    // Use BestEffort + KeepLast(1) to minimize SHM usage
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    
    if (message_size_ == "1KB") {
      // Create 1KB publishers
      for (int i = 0; i < num_topics_; i++) {
        int topic_id = node_num_ * num_topics_ + i;
        std::string topic_name = "test_topic_" + std::to_string(topic_id);
        auto pub = this->create_publisher<shm_tests::msg::FixedData1KB>(topic_name, qos);
        publishers_1kb_.push_back(pub);
      }
    } else if (message_size_ == "15KB") {
      // Create 15KB publishers
      for (int i = 0; i < num_topics_; i++) {
        int topic_id = node_num_ * num_topics_ + i;
        std::string topic_name = "test_topic_" + std::to_string(topic_id);
        auto pub = this->create_publisher<shm_tests::msg::FixedData15KB>(topic_name, qos);
        publishers_15kb_.push_back(pub);
      }
    }
    
    size_t pub_count = message_size_ == "1KB" ? publishers_1kb_.size() : publishers_15kb_.size();
    RCLCPP_INFO(this->get_logger(), "Created %zu publishers (%s, %s)", pub_count, message_size_.c_str(), mode.c_str());
    
    timer_ = this->create_wall_timer(
      0.1s, std::bind(&ManyTopicsTalker::timer_callback, this));
  }

private:
  void timer_callback() {
    // Reset timing accumulators for this cycle
    int64_t total_publish_duration_ns = 0;
    size_t publish_count = 0;

    if (message_size_ == "1KB") {
      publish_1kb(total_publish_duration_ns, publish_count);
    } else if (message_size_ == "15KB") {
      publish_15kb(total_publish_duration_ns, publish_count);
    }
    
    // Calculate average time per publish (convert ns to µs)
    double avg_time_per_topic_us = 0.0;
    if (publish_count > 0) {
      avg_time_per_topic_us = (total_publish_duration_ns / 1000.0) / publish_count;
    }
    
    // Throttle logging to reduce overhead
    std::string mode = zero_copy_ ? "zero-copy" : "standard";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Published to %zu topics: msg=%d, avg_per_topic=%.2f µs, size=%s (%s)",
                         publish_count, count_, avg_time_per_topic_us, message_size_.c_str(), mode.c_str());
    
    count_++;
  }
  
  void publish_1kb(int64_t& total_duration_ns, size_t& count) {
    if (zero_copy_) {
      // Zero-copy mode: Use loan API
      for (size_t i = 0; i < publishers_1kb_.size(); i++) {
        try {
          auto publish_start_ns = this->get_clock()->now().nanoseconds();
          
          // Borrow a loaned message from shared memory
          auto loaned_msg = publishers_1kb_[i]->borrow_loaned_message();
          
          // Write directly to shared memory (zero-copy!)
          auto& data = loaned_msg.get().data;
          
          // Write timestamp
          int64_t send_timestamp_ns = this->get_clock()->now().nanoseconds();
          std::memcpy(&data[0], &send_timestamp_ns, sizeof(send_timestamp_ns));
          
          // Write counter
          std::memcpy(&data[8], &count_, sizeof(count_));
          
          // Fill the payload (1024 - 12 = 1012 bytes)
          for (size_t j = 12; j < 1024; j++) {
            data[j] = static_cast<uint8_t>(j % 256);
          }
          
          // Publish the loaned message (no copy, just transfers ownership)
          publishers_1kb_[i]->publish(std::move(loaned_msg));
          
          auto publish_end_ns = this->get_clock()->now().nanoseconds();
          total_duration_ns += (publish_end_ns - publish_start_ns);
          count++;
          
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to borrow loaned message: %s", e.what());
        }
      }
    } else {
      // Standard mode: regular publish
      for (size_t i = 0; i < publishers_1kb_.size(); i++) {
        auto publish_start_ns = this->get_clock()->now().nanoseconds();
        
        auto message = shm_tests::msg::FixedData1KB();
        
        // Write timestamp
        int64_t send_timestamp_ns = this->get_clock()->now().nanoseconds();
        std::memcpy(&message.data[0], &send_timestamp_ns, sizeof(send_timestamp_ns));
        
        // Write counter
        std::memcpy(&message.data[8], &count_, sizeof(count_));
        
        // Fill the payload
        for (size_t j = 12; j < 1024; j++) {
          message.data[j] = static_cast<uint8_t>(j % 256);
        }
        
        publishers_1kb_[i]->publish(message);
        
        auto publish_end_ns = this->get_clock()->now().nanoseconds();
        total_duration_ns += (publish_end_ns - publish_start_ns);
        count++;
      }
    }
  }
  
  void publish_15kb(int64_t& total_duration_ns, size_t& count) {
    if (zero_copy_) {
      // Zero-copy mode: Use loan API
      for (size_t i = 0; i < publishers_15kb_.size(); i++) {
        try {
          auto publish_start_ns = this->get_clock()->now().nanoseconds();
          
          auto loaned_msg = publishers_15kb_[i]->borrow_loaned_message();
          auto& data = loaned_msg.get().data;
          
          // Write timestamp
          int64_t send_timestamp_ns = this->get_clock()->now().nanoseconds();
          std::memcpy(&data[0], &send_timestamp_ns, sizeof(send_timestamp_ns));
          
          // Write counter
          std::memcpy(&data[8], &count_, sizeof(count_));
          
          // Fill the payload (15368 - 12 = 15356 bytes)
          for (size_t j = 12; j < 15368; j++) {
            data[j] = static_cast<uint8_t>(j % 256);
          }
          
          publishers_15kb_[i]->publish(std::move(loaned_msg));
          
          auto publish_end_ns = this->get_clock()->now().nanoseconds();
          total_duration_ns += (publish_end_ns - publish_start_ns);
          count++;
          
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to borrow loaned message: %s", e.what());
        }
      }
    } else {
      // Standard mode: regular publish
      for (size_t i = 0; i < publishers_15kb_.size(); i++) {
        auto publish_start_ns = this->get_clock()->now().nanoseconds();
        
        auto message = shm_tests::msg::FixedData15KB();
        
        // Write timestamp
        int64_t send_timestamp_ns = this->get_clock()->now().nanoseconds();
        std::memcpy(&message.data[0], &send_timestamp_ns, sizeof(send_timestamp_ns));
        
        // Write counter
        std::memcpy(&message.data[8], &count_, sizeof(count_));
        
        // Fill the payload
        for (size_t j = 12; j < 15368; j++) {
          message.data[j] = static_cast<uint8_t>(j % 256);
        }
        
        publishers_15kb_[i]->publish(message);
        
        auto publish_end_ns = this->get_clock()->now().nanoseconds();
        total_duration_ns += (publish_end_ns - publish_start_ns);
        count++;
      }
    }
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<shm_tests::msg::FixedData1KB>::SharedPtr> publishers_1kb_;
  std::vector<rclcpp::Publisher<shm_tests::msg::FixedData15KB>::SharedPtr> publishers_15kb_;
  int count_;
  int node_num_;
  int num_topics_;
  std::string message_size_;
  bool zero_copy_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <node_num> [--ros-args --param name:=value ...]" << std::endl;
    std::cerr << "  node_num: integer identifier for this node (e.g., 0, 1, 2...)" << std::endl;
    std::cerr << "  ROS2 Parameters:" << std::endl;
    std::cerr << "    --param num_topics:=N (default: 10)" << std::endl;
    std::cerr << "    --param message_size:=SIZE (default: 1KB, options: 1KB, 15KB)" << std::endl;
    std::cerr << "    --param zero_copy:=true/false (default: true)" << std::endl;
    std::cerr << "Example: " << argv[0] << " 0 --ros-args --param num_topics:=10 --param message_size:=1KB --param zero_copy:=true" << std::endl;
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
