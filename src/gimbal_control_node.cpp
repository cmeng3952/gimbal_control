#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "gimbal/z2mini_controller_v2.hpp"
#include "gimbal/gimbal_mqtt_bridge.hpp"

using namespace gimbal;

static int getch_nonblock() {
  int ch = -1;
  termios oldt{}, newt{};
  if (tcgetattr(STDIN_FILENO, &oldt) != 0) return -1;
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  unsigned char c;
  if (read(STDIN_FILENO, &c, 1) == 1) ch = c;
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gimbal_control_node");

  // Gimbal IP/port
  std::string target_ip = "192.168.144.108";
  int target_port = 2337;
  std::string local_ip = "192.168.144.111";
  int local_port = 2338;

  node->declare_parameter<std::string>("target_ip", target_ip);
  node->declare_parameter<int>("target_port", target_port);
  node->declare_parameter<std::string>("local_ip", local_ip);
  node->declare_parameter<int>("local_port", local_port);
  node->get_parameter("target_ip", target_ip);
  node->get_parameter("target_port", target_port);
  node->get_parameter("local_ip", local_ip);
  node->get_parameter("local_port", local_port);

  auto controller = std::make_shared<Z2MiniControllerV2>(target_ip, static_cast<uint16_t>(target_port), local_ip, static_cast<uint16_t>(local_port));
  controller->center();

  // MQTT params
  std::string mqtt_broker = "tcp://47.104.183.127:1983";
  std::string mqtt_user = "pxtest";
  std::string mqtt_password = "test2025@px";
  std::string mqtt_sub_topic = "uavcontrol/gimbal/command/uav1";
  std::string mqtt_pub_topic = "uavcontrol/gimbal/state/uav1";

  node->declare_parameter<std::string>("mqtt_broker", mqtt_broker);
  node->declare_parameter<std::string>("mqtt_user", mqtt_user);
  node->declare_parameter<std::string>("mqtt_password", mqtt_password);
  node->declare_parameter<std::string>("mqtt_sub_topic", mqtt_sub_topic);
  node->declare_parameter<std::string>("mqtt_pub_topic", mqtt_pub_topic);
  node->get_parameter("mqtt_broker", mqtt_broker);
  node->get_parameter("mqtt_user", mqtt_user);
  node->get_parameter("mqtt_password", mqtt_password);
  node->get_parameter("mqtt_sub_topic", mqtt_sub_topic);
  node->get_parameter("mqtt_pub_topic", mqtt_pub_topic);

  GimbalMQTTBridge bridge(mqtt_broker, mqtt_user, mqtt_password, mqtt_sub_topic, mqtt_pub_topic);
  bridge.setController(controller);

  float angle_step = 5.0f;
  float fine_step = 1.0f;
  float fast_step = 15.0f;
  bool smooth_mode = false;

  RCLCPP_INFO(node->get_logger(), "Gimbal control node started. Keyboard optional: WASD pitch/yaw, QE roll, R center, Enter toggle smooth, +/- step size, 0 reset. Press ESC to exit.");

  rclcpp::Rate r(20.0);
  double last_pub = node->get_clock()->now().seconds();
  while (rclcpp::ok()) {
    const int k = getch_nonblock();
    if (k != -1) {
      float p, y, rl; std::tie(p, y, rl) = controller->currentAngles();
      auto step_for = [&](int key)->float { return (key >= 'A' && key <= 'Z') ? fast_step : angle_step; };
      if (k == 'w' || k == 'W') controller->setAngle(p + step_for(k), y, rl);
      else if (k == 's' || k == 'S') controller->setAngle(p - step_for(k), y, rl);
      else if (k == 'a' || k == 'A') controller->setAngle(p, y - step_for(k), rl);
      else if (k == 'd' || k == 'D') controller->setAngle(p, y + step_for(k), rl);
      else if (k == 'q' || k == 'Q') controller->setAngle(p, y, rl - step_for(k));
      else if (k == 'e' || k == 'E') controller->setAngle(p, y, rl + step_for(k));
      else if (k == 'r' || k == 'R') controller->center();
      else if (k == '+') { angle_step = std::min(30.0f, angle_step + 1.0f); fine_step = std::min(5.0f, fine_step + 0.5f); fast_step = std::min(45.0f, fast_step + 5.0f); }
      else if (k == '-') { angle_step = std::max(1.0f, angle_step - 1.0f); fine_step = std::max(0.5f, fine_step - 0.5f); fast_step = std::max(5.0f, fast_step - 5.0f); }
      else if (k == '0') { angle_step = 5.0f; fine_step = 1.0f; fast_step = 15.0f; }
      else if (k == 27 /* ESC */) break;
      else if (k == '\n' || k == '\r') { smooth_mode = !smooth_mode; }
    }

    const double now = node->get_clock()->now().seconds();
    if (now - last_pub > 1.0) {
      bridge.publishState();
      last_pub = now;
    }

    rclcpp::spin_some(node);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}


