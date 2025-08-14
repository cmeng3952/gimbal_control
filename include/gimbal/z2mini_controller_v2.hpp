#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "gimbal/gcu_packet_builder.hpp"
#include "gimbal/udp_connection_manager.hpp"

namespace gimbal {

class Z2MiniControllerV2 {
public:
  static constexpr float PITCH_MIN = -90.0f;
  static constexpr float PITCH_MAX = 30.0f;
  static constexpr float YAW_MIN = -180.0f;
  static constexpr float YAW_MAX = 180.0f;
  static constexpr float ROLL_MIN = -180.0f;
  static constexpr float ROLL_MAX = 180.0f;

  explicit Z2MiniControllerV2(const std::string& target_ip = "192.168.144.108",
                              uint16_t target_port = 2337,
                              const std::string& local_ip = "192.168.144.111",
                              uint16_t local_port = 2338)
      : target_ip_(target_ip), target_port_(target_port), local_ip_(local_ip), local_port_(local_port) {}

  bool setAngle(float pitch_deg, float yaw_deg, float roll_deg = 0.0f) {
    clampAngles(pitch_deg, yaw_deg, roll_deg);
    const auto pkt = packet_builder_.buildAngleControlPacket(pitch_deg, yaw_deg, roll_deg);
    const std::vector<uint8_t> bytes(pkt.begin(), pkt.end());
    const bool ok = udp_mgr_.sendPacket(bytes, target_ip_, target_port_, local_ip_, local_port_);
    if (ok) {
      current_pitch_ = pitch_deg;
      current_yaw_ = yaw_deg;
      current_roll_ = roll_deg;
    }
    return ok;
  }

  bool center() {
    // Ensure angle mode
    {
      const auto m = packet_builder_.buildModeSwitchPacket(GCUPacketBuilder::MODE_ANGLE);
      const std::vector<uint8_t> mb(m.begin(), m.end());
      udp_mgr_.sendPacket(mb, target_ip_, target_port_, local_ip_, local_port_);
    }

    // Send center command
    bool ok_center = false;
    {
      const auto pkt = packet_builder_.buildCenterPacket();
      const std::vector<uint8_t> bytes(pkt.begin(), pkt.end());
      ok_center = udp_mgr_.sendPacket(bytes, target_ip_, target_port_, local_ip_, local_port_);
    }

    // Follow with an explicit zero-angle set to ensure action
    bool ok_angle = setAngle(0.0f, 0.0f, 0.0f);

    if (ok_center || ok_angle) {
      current_pitch_ = 0.0f; current_yaw_ = 0.0f; current_roll_ = 0.0f;
      return true;
    }
    return false;
  }

  bool setAngleMode() {
    const auto pkt = packet_builder_.buildModeSwitchPacket(GCUPacketBuilder::MODE_ANGLE);
    const std::vector<uint8_t> bytes(pkt.begin(), pkt.end());
    return udp_mgr_.sendPacket(bytes, target_ip_, target_port_, local_ip_, local_port_);
  }

  bool rotateRelative(float dp, float dy, float dr = 0.0f) {
    return setAngle(current_pitch_ + dp, current_yaw_ + dy, current_roll_ + dr);
  }

  bool smoothRotateTo(float target_pitch, float target_yaw, float target_roll,
                      float duration_sec = 2.0f, int steps = 20) {
    clampAngles(target_pitch, target_yaw, target_roll);
    const float start_pitch = current_pitch_;
    const float start_yaw = current_yaw_;
    const float start_roll = current_roll_;

    const float step_delay = duration_sec / std::max(1, steps);

    for (int i = 0; i <= steps; ++i) {
      const float progress = static_cast<float>(i) / static_cast<float>(steps);
      const float t = 0.5f * (1.0f - std::cos(progress * static_cast<float>(M_PI)));
      const float p = start_pitch + (target_pitch - start_pitch) * t;
      const float y = start_yaw + (target_yaw - start_yaw) * t;
      const float r = start_roll + (target_roll - start_roll) * t;
      if (!setAngle(p, y, r)) return false;
      if (i < steps) std::this_thread::sleep_for(std::chrono::duration<double>(step_delay));
    }
    return true;
  }

  std::tuple<float, float, float> currentAngles() const { return {current_pitch_, current_yaw_, current_roll_}; }

private:
  static void clampAngles(float& pitch, float& yaw, float& roll) {
    pitch = std::max(PITCH_MIN, std::min(PITCH_MAX, pitch));
    yaw = std::max(YAW_MIN, std::min(YAW_MAX, yaw));
    roll = std::max(ROLL_MIN, std::min(ROLL_MAX, roll));
  }

  std::string target_ip_;
  uint16_t target_port_;
  std::string local_ip_;
  uint16_t local_port_;

  GCUPacketBuilder packet_builder_{};
  UDPConnectionManager udp_mgr_{};

  float current_pitch_ = 0.0f;
  float current_yaw_ = 0.0f;
  float current_roll_ = 0.0f;
};

}  // namespace gimbal


