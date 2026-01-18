#include "rclcpp/rclcpp.hpp"
#include "shm_tests/msg/fixed_data1_kb.hpp"
#include "shm_tests/msg/fixed_data15_kb.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <thread>

static std::atomic<uint64_t> total_latency_us{0};
static std::atomic<uint64_t> total_messages{0};
static std::atomic<bool> stop_requested{false};

extern "C" void signal_handler(int sig) {
  std::cerr << "[DEBUG] Signal " << sig << " received, setting stop_requested\n" << std::flush;
  stop_requested.store(true, std::memory_order_relaxed);
}

class ManyTopicsListener : public rclcpp::Node {
public:
  explicit ManyTopicsListener(int node_num)
  : Node("many_topics_listener"),
    node_num_(node_num)
  {
    this->declare_parameter<int>("num_topics", 10);
    this->declare_parameter<std::string>("message_size", "1KB");  // "1KB" or "15KB"
    this->declare_parameter<bool>("zero_copy", true);

    num_topics_      = this->get_parameter("num_topics").as_int();
    message_size_    = this->get_parameter("message_size").as_string();
    zero_copy_       = this->get_parameter("zero_copy").as_bool();

    const std::string mode = zero_copy_ ? "zero-copy" : "standard";

    RCLCPP_INFO(get_logger(),
      "Node %d: Creating %d subscriptions with %s messages (%s)",
      node_num_, num_topics_, message_size_.c_str(), mode.c_str());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    
    if (message_size_ == "1KB") {
      subscriptions_1kb_.reserve(num_topics_);
      for (int i = 0; i < num_topics_; ++i) {
        int topic_id = node_num_ * num_topics_ + i;
        std::string topic_name = "test_topic_" + std::to_string(topic_id);
        auto sub = create_subscription<shm_tests::msg::FixedData1KB>(
          topic_name, qos,
          [this](const shm_tests::msg::FixedData1KB::SharedPtr msg) {
            const int64_t receive_time_ns = this->get_clock()->now().nanoseconds();
            int64_t send_timestamp_ns = 0;
            std::memcpy(&send_timestamp_ns, &msg->data[0], sizeof(send_timestamp_ns));
            const int64_t latency_ns = receive_time_ns - send_timestamp_ns;
            const uint64_t latency_us_local = static_cast<uint64_t>(latency_ns / 1000);

            total_latency_us.fetch_add(latency_us_local, std::memory_order_relaxed);
            total_messages.fetch_add(1, std::memory_order_relaxed);
          });
        subscriptions_1kb_.push_back(sub);
      }
    } else if (message_size_ == "15KB") {
      subscriptions_15kb_.reserve(num_topics_);
      for (int i = 0; i < num_topics_; ++i) {
        int topic_id = node_num_ * num_topics_ + i;
        std::string topic_name = "test_topic_" + std::to_string(topic_id);
        auto sub = create_subscription<shm_tests::msg::FixedData15KB>(
          topic_name, qos,
          [this](const shm_tests::msg::FixedData15KB::SharedPtr msg) {
            const int64_t receive_time_ns = this->get_clock()->now().nanoseconds();
            int64_t send_timestamp_ns = 0;
            std::memcpy(&send_timestamp_ns, &msg->data[0], sizeof(send_timestamp_ns));
            const int64_t latency_ns = receive_time_ns - send_timestamp_ns;
            const uint64_t latency_us_local = static_cast<uint64_t>(latency_ns / 1000);

            total_latency_us.fetch_add(latency_us_local, std::memory_order_relaxed);
            total_messages.fetch_add(1, std::memory_order_relaxed);
          });
        subscriptions_15kb_.push_back(sub);
      }
    }
  }

private:
  int node_num_{0};
  int num_topics_{0};
  std::string message_size_;
  bool zero_copy_{false};

  std::vector<rclcpp::Subscription<shm_tests::msg::FixedData1KB>::SharedPtr> subscriptions_1kb_;
  std::vector<rclcpp::Subscription<shm_tests::msg::FixedData15KB>::SharedPtr> subscriptions_15kb_;
};


int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <node_num> [--ros-args --param ...]\n";
    return 1;
  }

  const int node_num = std::atoi(argv[1]);
  
  // Install our signal handlers BEFORE rclcpp::init to override ROS2's signal handling
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  auto node = std::make_shared<ManyTopicsListener>(node_num);
  
  // Use MultiThreadedExecutor for better performance
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 
                                                  std::thread::hardware_concurrency());
  exec.add_node(node);

  // Spin in a separate thread
  std::thread spin_thread([&exec]() {
    while (!stop_requested.load(std::memory_order_relaxed)) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  // Wait for stop signal
  while (!stop_requested.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Stop spinning
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  std::cerr << "[DEBUG] Loop exited, printing stats\n" << std::flush;

  // Print final statistics when stopping
  const uint64_t msgs = total_messages.load(std::memory_order_relaxed);
  const uint64_t sum_us = total_latency_us.load(std::memory_order_relaxed);

  std::cerr << "[DEBUG] msgs=" << msgs << ", sum_us=" << sum_us << "\n" << std::flush;

  if (msgs > 0) {
    const double avg_us = static_cast<double>(sum_us) / static_cast<double>(msgs);
    std::cerr << "Stats: total_msgs=" << msgs << ", Latency: avg=" << avg_us << " µs\n" << std::flush;
    std::cout << "Stats: total_msgs=" << msgs << ", Latency: avg=" << avg_us << " µs\n" << std::flush;
  } else {
    std::cerr << "No messages received.\n" << std::flush;
    std::cout << "No messages received.\n" << std::flush;
  }

  exec.remove_node(node);
  rclcpp::shutdown();

  return 0;
}
