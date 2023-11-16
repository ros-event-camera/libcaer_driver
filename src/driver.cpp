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
  messageThresholdTime_ = uint64_t(std::abs(get_or("event_message_time_threshold", 1.0e-3)) * 1e9);
  messageThresholdSize_ =
    static_cast<size_t>(std::abs(get_or("event_message_size_threshold", int64_t(1000000000))));

  eventPub_ = this->create_publisher<EventPacketMsg>(
    "~/events", rclcpp::QoS(rclcpp::KeepLast(get_or("send_queue_size", 1000)))
                  .best_effort()
                  .durability_volatile());

  const std::string deviceType = get_or("device_type", std::string("davis"));
  try {
    wrapper_.reset(new LibcaerWrapper());
    wrapper_->setCallbackHandler(this);
    wrapper_->initialize(deviceType, get_or("device_id", 1), get_or("serial", std::string()));
  } catch (std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "sensor initialization failed: " << e.what());
    throw(e);
  }

  declareParameters();

  isBigEndian_ = check_endian::isBigEndian();
  // ------ get other parameters from camera
  imageMsg_.width = wrapper_->getWidth();
  imageMsg_.height = wrapper_->getHeight();
  imageMsg_.header.frame_id = get_or("frame_id", wrapper_->getSerialNumber());
  RCLCPP_INFO_STREAM(
    get_logger(), "res: " << imageMsg_.width << " x " << imageMsg_.height
                          << " using frame id: " << imageMsg_.header.frame_id);

  infoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, get_name(), get_or("camerainfo_url", std::string("")));

  cameraInfoMsg_ = infoManager_->getCameraInfo();
  if (
    ((cameraInfoMsg_.width != 0) && (cameraInfoMsg_.width != imageMsg_.width)) ||
    ((cameraInfoMsg_.height != 0) && (cameraInfoMsg_.height != imageMsg_.height))) {
    RCLCPP_WARN(get_logger(), "sensor resolution does not match calibration file!");
  }
  cameraInfoMsg_.width = imageMsg_.width;
  cameraInfoMsg_.height = imageMsg_.height;
  cameraInfoMsg_.header.frame_id = imageMsg_.header.frame_id;

  cameraPub_ =
    image_transport::create_camera_publisher(this, "~/image_raw", rmw_qos_profile_sensor_data);

  start();
}

Driver::~Driver()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

void Driver::declareParameters()
{
  const auto & parameters = wrapper_->getParameters();
  for (const auto & p : parameters) {
    const auto & pi = p.second;  // parameter info
    parameters_.insert(
      {p.first, Parameter(p.first, Parameter::INTEGER, pi.defVal, pi.minVal, pi.maxVal)});
    RCLCPP_INFO_STREAM(
      get_logger(), p.first << " " << pi.defVal << " " << pi.minVal << " " << pi.maxVal);
  }
  for (auto & pi : parameters_) {
    auto & p = pi.second;
    if (p.type == Parameter::INTEGER) {
      try {
        int v(0);
        if (has_parameter(p.name)) {
          try {
            v = get_parameter(p.name).as_int();
          } catch (const rclcpp::ParameterTypeException & e) {
            v = p.value.intValue;
            RCLCPP_WARN_STREAM(get_logger(), "ignoring param " << p.name << " with invalid type!");
          }
        } else {
          v = declare_parameter(
            p.name, p.value.intValue, rcl_interfaces::msg::ParameterDescriptor(), false);
        }
        p.value.intValue = std::clamp<int>(v, p.min_value.intValue, p.max_value.intValue);
        if (p.value.intValue != v) {
          RCLCPP_INFO_STREAM(
            get_logger(),
            p.name << " outside limits, adjusted " << v << " -> " << p.value.intValue);
          set_parameter(rclcpp::Parameter(p.name, p.value.intValue));
        } else {
          RCLCPP_INFO_STREAM(
            get_logger(), "parameter " << p.name << " initialized with value " << p.value.intValue);
        }
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        RCLCPP_WARN_STREAM(
          get_logger(), "overwriting bad param with default: " + std::string(e.what()));
        declare_parameter(
          p.name, p.value.intValue, rcl_interfaces::msg::ParameterDescriptor(), true);
      }
    }
  }
}

void Driver::updateParameter(Parameter * p, const rcl_interfaces::msg::ParameterValue & rp)
{
  try {
    switch (p->type) {
      case Parameter::INTEGER: {
        if (has_parameter(p->name)) {
          p->value.intValue =
            std::clamp<int>(rp.integer_value, p->min_value.intValue, p->max_value.intValue);
          if (p->value.intValue != rp.integer_value) {
            RCLCPP_WARN_STREAM(
              get_logger(), p->name << ": " << rp.integer_value << " out of range, adjusted to "
                                    << p->value.intValue);
          }
          // now set the parameter in ROS land
          if (wrapper_) {
            const int v_new = wrapper_->setParameter(p->name, p->value.intValue);
            if (v_new != p->value.intValue) {
              RCLCPP_WARN_STREAM(get_logger(), "libcaer adjusted " << p->name << " to " << v_new);
              p->value.intValue = v_new;
            }
          }
          if (rp.integer_value != p->value.intValue) {
            // only communicate the parameter changes to ROS if there actually
            // was a change, or else this triggers an infinite sequence of callbacks
            set_parameter(rclcpp::Parameter(p->name, p->value.intValue));
          }
        }
        break;
      }
      default:
        throw(std::runtime_error("invalid parameter type!"));
    }
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_WARN_STREAM(get_logger(), "ignoring param  " << p->name << " with invalid type!");
  }
}

void Driver::onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event)
{
  if (event->node != this->get_fully_qualified_name()) {
    return;
  }

  std::vector<std::string> validEvents;
  for (const auto & p : parameters_) {
    validEvents.push_back(p.first);
  }
  rclcpp::ParameterEventsFilter filter(
    event, validEvents, {rclcpp::ParameterEventsFilter::EventType::CHANGED});
  for (auto & ev_it : filter.get_events()) {
    const std::string & name = ev_it.second->name;
    auto it = parameters_.find(name);
    if (it != parameters_.end()) {
      updateParameter(&(it->second), ev_it.second->value);
    }
  }
}

void Driver::start()
{
  double printInterval;
  this->get_parameter_or("statistics_print_interval", printInterval, 1.0);
  wrapper_->setStatisticsInterval(printInterval);

  // ------ start camera, may get callbacks from then on
  wrapper_->startSensor();

#if 0
  parameterSubscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this->get_node_topics_interface(),
    std::bind(&Driver::onParameterEvent, this, std::placeholders::_1));
#else
  parameterSubscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this, std::bind(&Driver::onParameterEvent, this, std::placeholders::_1));
#endif
}

bool Driver::stop()
{
  RCLCPP_INFO(get_logger(), "driver stopping sensor");
  if (wrapper_) {
    wrapper_->stopSensor();
    return (true);
  }
  return false;
}

void Driver::configureSensor() {}

void Driver::polarityPacketCallback(uint64_t t, const libcaer::events::PolarityEventPacket & packet)
{
  if (eventPub_->get_subscription_count() > 0) {
    if (!msg_) {
      msg_.reset(new EventPacketMsg());
      msg_->header.frame_id = imageMsg_.header.frame_id;
      msg_->time_base = packet[0].getTimestamp64(packet);
      msg_->encoding = "mono";
      msg_->seq = seq_++;
      msg_->width = imageMsg_.width;
      msg_->height = imageMsg_.height;
      msg_->is_bigendian = isBigEndian_;
      msg_->header.stamp = rclcpp::Time(t, RCL_SYSTEM_TIME);
      msg_->events.reserve(reserveSize_);
    }
    auto & events = msg_->events;
    (void)LibcaerWrapper::convert_to_mono(&events, msg_->time_base, packet);
    if (t - lastMessageTime_ > messageThresholdTime_ || events.size() > messageThresholdSize_) {
      reserveSize_ = std::max(reserveSize_, events.size());
      eventPub_->publish(std::move(msg_));
      lastMessageTime_ = t;
      wrapper_->updateBytesSent(events.size());
      wrapper_->updateMsgsSent(1);
    } else {
      if (!msg_) {
        msg_.reset();
      }
    }
  }
}

void Driver::framePacketCallback(uint64_t t, const libcaer::events::FrameEventPacket & packet)
{
  if (cameraPub_.getNumSubscribers() > 0) {
    imageMsg_.header.stamp = rclcpp::Time(t, RCL_SYSTEM_TIME);
    sensor_msgs::msg::CameraInfo::UniquePtr cinfo(new sensor_msgs::msg::CameraInfo(cameraInfoMsg_));
    sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image(imageMsg_));
    void frame_to_ros_msg(
      const libcaer::events::FrameEventPacket & packet, std::string * encoding,
      std::vector<uint8_t> * out, uint32_t * height, uint32_t * width, uint32_t * step);

    LibcaerWrapper::frame_to_ros_msg(
      packet, &img->encoding, &img->data, &img->width, &img->height, &img->step);
    cameraPub_.publish(std::move(img), std::move(cinfo));
  }
}
}  // namespace libcaer_driver

RCLCPP_COMPONENTS_REGISTER_NODE(libcaer_driver::Driver)
