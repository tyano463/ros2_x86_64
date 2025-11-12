#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DuplexNode : public rclcpp::Node {
public:
  DuplexNode(const std::string &node_name,
             const std::string &send_topic,
             const std::string &recv_topic)
      : Node(node_name), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>(send_topic, 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        recv_topic, 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        });
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          auto msg = std_msgs::msg::String();
          msg.data = "Message " + std::to_string(count_) + " from " + this->get_name();
          publisher_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "Sent: '%s'", msg.data.c_str());
          count_++;
        });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  if (argc != 4) {
    std::cerr << "Usage: duplex_node <node_name> <send_topic> <recv_topic>\n";
    return 1;
  }
  auto node = std::make_shared<DuplexNode>(argv[1], argv[2], argv[3]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

