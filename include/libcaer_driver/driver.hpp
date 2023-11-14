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

#include <camera_info_manager/camera_info_manager.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <image_transport/image_transport.hpp>
#include <libcaercpp/events/frame.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "libcaer_driver/callback_handler.hpp"
#include "libcaer_driver/resize_hack.hpp"

namespace libcaer_driver
{
class LibcaerWrapper;  // forward decl

class Driver : public rclcpp::Node, public CallbackHandler
{
  using EventPacketMsg = event_camera_msgs::msg::EventPacket;
  using Trigger = std_srvs::srv::Trigger;

public:
  explicit Driver(const rclcpp::NodeOptions & options);
  ~Driver();

  // ---------------- inherited from CallbackHandler -----------
  void polarityPacketCallback(
    uint64_t t, const libcaer::events::PolarityEventPacket & packet) override;
  void framePacketCallback(uint64_t t, const libcaer::events::FrameEventPacket & packet) override;

  // ---------------- end of inherited  -----------

private:
  // related to dynanmic config (runtime parameter update)
  rcl_interfaces::msg::SetParametersResult parameterChanged(
    const std::vector<rclcpp::Parameter> & params);
  void onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event);
  // misc helper functions
  void start();
  bool stop();
  void configureSensor();
  template <class T>
  T get_or(const std::string & name, const T & def)
  {
    T p;
    (void)get_parameter_or(name, p, def);
    return (p);
  }

  // ------------------------  variables ------------------------------
  std::shared_ptr<LibcaerWrapper> wrapper_;
  bool isBigEndian_;
  uint64_t seq_{0};        // sequence number
  size_t reserveSize_{0};  // recommended reserve size
  uint64_t lastMessageTime_{0};
  uint64_t messageThresholdTime_{0};  // threshold time for sending message
  size_t messageThresholdSize_{0};    // threshold size for sending message
  EventPacketMsg::UniquePtr msg_;
  rclcpp::Publisher<EventPacketMsg>::SharedPtr eventPub_;
  // ------ related to dynamic config and services
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callbackHandle_;
  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent, std::allocator<void>>>
    parameterSubscription_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  image_transport::CameraPublisher cameraPub_;
  sensor_msgs::msg::Image imageMsg_;
  sensor_msgs::msg::CameraInfo cameraInfoMsg_;
};
}  // namespace libcaer_driver
#endif  // LIBCAER_DRIVER__DRIVER_HPP_
