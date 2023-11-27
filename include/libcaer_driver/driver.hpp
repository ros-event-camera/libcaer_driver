// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIBCAER_DRIVER__DRIVER_HPP_
#define LIBCAER_DRIVER__DRIVER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <image_transport/image_transport.hpp>
#include <libcaercpp/events/frame.hpp>
#include <libcaercpp/events/imu6.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "libcaer_driver/callback_handler.hpp"
#include "libcaer_driver/parameter.hpp"

namespace libcaer_driver
{
class LibcaerWrapper;  // forward decl

class Driver : public rclcpp::Node, public CallbackHandler
{
  using EventPacketMsg = event_camera_msgs::msg::EventPacket;
  using ImuMsg = sensor_msgs::msg::Imu;
  using TimeMsg = builtin_interfaces::msg::Time;
  using Trigger = std_srvs::srv::Trigger;

public:
  explicit Driver(const rclcpp::NodeOptions & options);
  ~Driver();

  // ---------------- inherited from CallbackHandler -----------
  void declareParameter(const std::shared_ptr<Parameter> &p, const RosParameter &rp) override;
  void polarityPacketCallback(
    uint64_t t, const libcaer::events::PolarityEventPacket & packet) override;
  void framePacketCallback(uint64_t t, const libcaer::events::FrameEventPacket & packet) override;
  void imu6PacketCallback(uint64_t t, const libcaer::events::IMU6EventPacket & packet) override;

  // ---------------- end of inherited  -----------

private:
  // related to dynamic config (runtime parameter update)
  void onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event);
  // misc helper functions
  void start();
  bool stop();
  void configureSensor();
  void resetTime();
  void resetMsg(TimeMsg::ConstSharedPtr msg);

  void updateParameter(
  const std::string & name, const std::shared_ptr<Parameter> p, const rclcpp::ParameterValue & rp);

  template <typename T>
  T get_or(const std::string & name, const T & def)
  {
    T p;
    (void)get_parameter_or(name, p, def);
    return (p);
  }

  // ------------------------  variables ------------------------------
  std::shared_ptr<LibcaerWrapper> wrapper_;
  bool isMaster_{true};
  bool isBigEndian_;
  std::string cameraFrameId_{"camera"};
  std::string imuFrameId_{"imu"};
  uint32_t dvsWidth_{0};
  uint32_t dvsHeight_{0};
  uint32_t apsWidth_{0};
  uint32_t apsHeight_{0};
  uint64_t seq_{0};        // sequence number
  size_t reserveSize_{0};  // recommended reserve size
  uint64_t lastMessageTime_{0};
  uint64_t messageThresholdTime_{0};  // threshold time for sending message
  size_t messageThresholdSize_{0};    // threshold size for sending message
  EventPacketMsg::UniquePtr eventMsg_;
  rclcpp::Publisher<EventPacketMsg>::SharedPtr eventPub_;
  rclcpp::Publisher<ImuMsg>::SharedPtr imuPub_;
  rclcpp::Publisher<TimeMsg>::SharedPtr resetPub_;
  rclcpp::Subscription<TimeMsg>::SharedPtr resetSub_;
  // ------ related to dynamic config and services
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterSubscription_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  image_transport::CameraPublisher cameraPub_;
  sensor_msgs::msg::CameraInfo cameraInfoMsg_;
  std::map<std::string, std::shared_ptr<Parameter>> parameterMap_; 
};
}  // namespace libcaer_driver
#endif  // LIBCAER_DRIVER__DRIVER_HPP_
