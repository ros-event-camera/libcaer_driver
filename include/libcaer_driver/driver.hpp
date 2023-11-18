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

namespace detail
{
// declare a no-op base template here. The specializations are in the cpp file.
template <class T>
T sendParameterChange(LibcaerWrapper *, const std::string &, const Parameter &, const T &)
{
  return (T());
}
}  // namespace detail

class Driver : public rclcpp::Node, public CallbackHandler
{
  using EventPacketMsg = event_camera_msgs::msg::EventPacket;
  using ImuMsg = sensor_msgs::msg::Imu;
  using Trigger = std_srvs::srv::Trigger;

public:
  explicit Driver(const rclcpp::NodeOptions & options);
  ~Driver();

  // ---------------- inherited from CallbackHandler -----------
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
  void declareParameters();
  void applyParameters();
  void resetBaseTime();

  void updateParameter(
    const std::string & name, const Parameter & p, const rclcpp::ParameterValue & rp);

  template <typename T>
  T get_or(const std::string & name, const T & def)
  {
    T p;
    (void)get_parameter_or(name, p, def);
    return (p);
  }

  // returns the current value (either default, or value overridden by user at node start)
  template <class T>
  T declareParameter(const std::string & name, const Parameter & p)
  {
    T v(p.defVal.get<T>());
    try {
      T rawV(v);
      if (this->has_parameter(name)) {
        try {
          rawV = this->get_parameter(name).get_value<T>();  // get user-provided ROS value
        } catch (const rclcpp::ParameterTypeException & e) {
          RCLCPP_WARN_STREAM(get_logger(), "ignoring param " << name << " with invalid type!");
        }
      } else {
        rawV = this->declare_parameter(
          name, p.defVal.get<T>(), rcl_interfaces::msg::ParameterDescriptor(), false);
      }
      v = std::clamp<T>(rawV, p.minVal.get<T>(), p.maxVal.get<T>());
      if (rawV != v) {
        RCLCPP_INFO_STREAM(
          get_logger(), name << " outside limits, adjusted " << rawV << " -> " << v);
        this->set_parameter(rclcpp::Parameter(name, v));
      } else {
        RCLCPP_INFO_STREAM(get_logger(), "parameter " << name << " initialized with value " << v);
      }
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
      RCLCPP_WARN_STREAM(
        get_logger(), "overwriting bad param with default: " + std::string(e.what()));
      this->declare_parameter(name, v, rcl_interfaces::msg::ParameterDescriptor(), true);
    }
    return (v);
  }

  template <class T>
  T setParameter(const std::string & name, const Parameter & p, const T & targetValue)
  {
    T v(targetValue);
    if (this->has_parameter(name)) {
      v = std::clamp<T>(targetValue, p.minVal.get<T>(), p.maxVal.get<T>());
      if (v != targetValue) {
        RCLCPP_WARN_STREAM(
          get_logger(), name << ": " << targetValue << " out of range, adjusted to " << v);
      }
      // now update the parameter
      if (wrapper_) {
        //        const T v_new = wrapper_->setParameter<T>(name, p, v);
        const T v_new = detail::sendParameterChange<T>(wrapper_.get(), name, p, v);
        if (v_new != v) {
          RCLCPP_WARN_STREAM(get_logger(), "libcaer adjusted " << name << " to " << v_new);
        }
        v = v_new;
      }
      if (targetValue != v) {
        // only communicate the parameter changes to ROS if there actually
        // was a change, or else this triggers an infinite sequence of callbacks
        this->set_parameter(rclcpp::Parameter(name, v));  // this updates the ROS param
      }
    }
    return (v);
  }

  // ------------------------  variables ------------------------------
  std::shared_ptr<LibcaerWrapper> wrapper_;
  bool isBigEndian_;
  std::string cameraFrameId_{"camera"};
  std::string imuFrameId_{"imu"};
  uint32_t width_{0};
  uint32_t height_{0};
  uint64_t seq_{0};        // sequence number
  size_t reserveSize_{0};  // recommended reserve size
  uint64_t lastMessageTime_{0};
  uint64_t messageThresholdTime_{0};  // threshold time for sending message
  size_t messageThresholdSize_{0};    // threshold size for sending message
  EventPacketMsg::UniquePtr eventMsg_;
  rclcpp::Time rosBaseTime_;
  rclcpp::Publisher<EventPacketMsg>::SharedPtr eventPub_;
  rclcpp::Publisher<ImuMsg>::SharedPtr imuPub_;
  // ------ related to dynamic config and services
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterSubscription_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  image_transport::CameraPublisher cameraPub_;
  sensor_msgs::msg::CameraInfo cameraInfoMsg_;
};
}  // namespace libcaer_driver
#endif  // LIBCAER_DRIVER__DRIVER_HPP_
