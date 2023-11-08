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

#include "libcaer_driver/driver.hpp"

#include <chrono>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <map>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

#include "libcaer_driver/check_endian.hpp"
#include "libcaer_driver/libcaer_wrapper.hpp"

namespace libcaer_driver
{
Driver::Driver(const rclcpp::NodeOptions & options)
: Node(
    "libcaer_driver",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  double mtt;
  this->get_parameter_or("event_message_time_threshold", mtt, 1e-3);
  messageThresholdTime_ = uint64_t(std::abs(mtt) * 1e9);
  int64_t mts;
  this->get_parameter_or("event_message_size_threshold", mts, int64_t(1000000000));
  messageThresholdSize_ = static_cast<size_t>(std::abs(mts));

  int qs;
  this->get_parameter_or("send_queue_size", qs, 1000);
  eventPub_ = this->create_publisher<EventPacketMsg>(
    "~/events", rclcpp::QoS(rclcpp::KeepLast(qs)).best_effort().durability_volatile());
  std::string deviceType;
  get_parameter_or("device_type", deviceType, std::string("dvs128"));
  std::string serial;
  get_parameter_or("serial", serial, std::string());
  int deviceId;
  get_parameter_or("device_id", deviceId, 1);
  try {
    wrapper_.reset(new LibcaerWrapper());
    wrapper_->initialize(deviceType, deviceId, serial);
  } catch (std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "sensor initialization failed: " << e.what());
    throw(e);
  }
  start();
}

Driver::~Driver()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

rcl_interfaces::msg::SetParametersResult Driver::parameterChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = false;
  res.reason = "not set";
  for (const auto & p : params) {
    RCLCPP_INFO_STREAM(get_logger(), "parameter changed: " << p.get_name());
  }
  return (res);
}

void Driver::onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event)
{
  if (event->node != this->get_fully_qualified_name()) {
    return;
  }
}
void Driver::start()
{
  double printInterval;
  this->get_parameter_or("statistics_print_interval", printInterval, 1.0);
  wrapper_->setStatisticsInterval(printInterval);
  if (frameId_.empty()) {
    // default frame id to last 4 digits of serial number
    frameId_ = wrapper_->getSerialNumber();
  }

  RCLCPP_INFO_STREAM(get_logger(), "using frame id: " << frameId_);

  // ------ get other parameters from camera
  width_ = wrapper_->getWidth();
  height_ = wrapper_->getHeight();
  isBigEndian_ = check_endian::isBigEndian();

  // ------ start camera, may get callbacks from then on
  (void)wrapper_->startSensor();

  callbackHandle_ = this->add_on_set_parameters_callback(
    std::bind(&Driver::parameterChanged, this, std::placeholders::_1));
  parameterSubscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this->get_node_topics_interface(),
    std::bind(&Driver::onParameterEvent, this, std::placeholders::_1));
}

bool Driver::stop()
{
  if (wrapper_) {
    return (wrapper_->stopSensor());
  }
  return false;
}

void Driver::configureSensor() {}

void Driver::rawDataCallback(uint64_t t, const uint8_t * start, const uint8_t * end)
{
  if (eventPub_->get_subscription_count() > 0) {
    if (!msg_) {
      msg_.reset(new EventPacketMsg());
      msg_->header.frame_id = frameId_;
      msg_->time_base = 0;  // not used here
      msg_->encoding = "mono";
      msg_->seq = seq_++;
      msg_->width = width_;
      msg_->height = height_;
      msg_->header.stamp = rclcpp::Time(t, RCL_SYSTEM_TIME);
      msg_->events.reserve(reserveSize_);
    }
    const size_t n = end - start;
    auto & events = msg_->events;
    const size_t oldSize = events.size();
    resize_hack(events, oldSize + n);
    memcpy(reinterpret_cast<void *>(events.data() + oldSize), start, n);

    if (t - lastMessageTime_ > messageThresholdTime_ || events.size() > messageThresholdSize_) {
      reserveSize_ = std::max(reserveSize_, events.size());
      eventPub_->publish(std::move(msg_));
      lastMessageTime_ = t;
      wrapper_->updateBytesSent(events.size());
      wrapper_->updateMsgsSent(1);
    }
  } else {
    if (msg_) {
      msg_.reset();
    }
  }
}
}  // namespace libcaer_driver

RCLCPP_COMPONENTS_REGISTER_NODE(libcaer_driver::Driver)
