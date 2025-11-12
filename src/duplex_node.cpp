#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <string>
#include <chrono>
#include <iostream>
#include <netinet/in.h>
#include <unistd.h>

#define NODE_NAME "car_navi"

template<typename... Args>
void log_impl(const char* file, int line, const char* func, const Args&... args) {
    std::cout << file << "(" << line << ") " << func << ": ";
    (std::cout << ... << args) << std::endl;
}

#define d(...) log_impl(__FILE__, __LINE__, __func__, __VA_ARGS__)

class MultiThreadedNode : public rclcpp::Node {
public:
  MultiThreadedNode() : Node(NODE_NAME), shared_message_(""), stop_flag_(false) {
    // ROS 2受信（例：/commandsトピック）
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "commands", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push(msg->data);
      });

    // スレッド起動
    sender_thread_ = std::thread(&MultiThreadedNode::senderLoop, this);
    tcp_thread_ = std::thread(&MultiThreadedNode::tcpServerLoop, this);
    d("thread spawned");
    mainLoop();  // メインスレッドは直接呼び出す
  }

  ~MultiThreadedNode() {
    stop_flag_ = true;
    sender_cv_.notify_all();
    if (sender_thread_.joinable()) sender_thread_.join();
    if (tcp_thread_.joinable()) tcp_thread_.join();
  }

private:
  // メインスレッド：キュー処理と送信通知
  void mainLoop() {
    while (rclcpp::ok() && !stop_flag_) {
      rclcpp::spin_some(this->get_node_base_interface());

      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!command_queue_.empty()) {
          std::string cmd = command_queue_.front();
          command_queue_.pop();
          shared_message_ = "Processed: " + cmd;
          sender_cv_.notify_one();  // 送信スレッドを起こす
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // 送信スレッド：通知で起きて処理
  void senderLoop() {
    while (!stop_flag_) {
      std::unique_lock<std::mutex> lock(sender_mutex_);
      sender_cv_.wait(lock);  // 通知が来るまで待機

      if (stop_flag_) break;

      std::lock_guard<std::mutex> qlock(queue_mutex_);
      std::cout << "[Sender] " << shared_message_ << std::endl;
    }
  }

  // TCPコマンド受付スレッド
  void tcpServerLoop() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(12345);

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 3);

    while (!stop_flag_) {
      new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
      int valread = read(new_socket, buffer, 1024);
      std::string cmd(buffer, valread);

      if (cmd == "ping" || cmd == "start") {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push(cmd);
      } else {
	      std::cout << "unexpected cmd received: " << cmd << std::endl;
      }

      close(new_socket);
    }

    close(server_fd);
  }

  // ROS 2受信
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // スレッドと同期
  std::thread sender_thread_;
  std::thread tcp_thread_;
  std::mutex sender_mutex_;
  std::condition_variable sender_cv_;

  std::mutex queue_mutex_;
  std::queue<std::string> command_queue_;
  std::string shared_message_;
  bool stop_flag_;
};

int main(int argc, char *argv[]) {
	d("");
  rclcpp::init(argc, argv);
	d("");
  auto node = std::make_shared<MultiThreadedNode>();
  rclcpp::shutdown();
  return 0;
}

