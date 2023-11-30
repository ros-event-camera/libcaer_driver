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

#include <chrono>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <libcaer_driver/check_endian.hpp>
#include <libcaer_driver/driver.hpp>
#include <libcaer_driver/libcaer_wrapper.hpp>
#include <libcaer_driver/message_converter.hpp>
#include <map>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

namespace libcaer_driver
{
template <class T>
void set_range(
  rcl_interfaces::msg::ParameterDescriptor::_floating_point_range_type *,
  rcl_interfaces::msg::ParameterDescriptor::_integer_range_type *, const T &, const T &, const T &)
{
}

template <>
void set_range<int32_t>(
  rcl_interfaces::msg::ParameterDescriptor::_floating_point_range_type *,
  rcl_interfaces::msg::ParameterDescriptor::_integer_range_type * irange, const int32_t & nv,
  const int32_t & xv, const int32_t & step)
{
  rcl_interfaces::msg::IntegerRange ir;
  ir.from_value = nv;
  ir.to_value = xv;
  ir.step = step;
  irange->push_back(ir);
}

// returns the current value (either default, or value overridden by user at node start)
template <class T>
static T declare_ros_parameter(Driver * node, const RosParameter & p)
{
  ValueWithLimits vvl = p.value;
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = p.name;
  desc.description = p.desc;
  T v(vvl.curVal.get<T>());
  try {
    T rawV(v);
    if (node->has_parameter(p.name)) {
      try {
        rawV = node->get_parameter(p.name).get_value<T>();  // get user-provided ROS value
      } catch (const rclcpp::ParameterTypeException & e) {
        RCLCPP_WARN_STREAM(
          node->get_logger(), "ignoring param " << p.name << " with invalid type!");
      }
    } else {
      set_range<T>(
        &desc.floating_point_range, &desc.integer_range, vvl.minVal.get<T>(), vvl.maxVal.get<T>(),
        1);
      rawV = node->declare_parameter(p.name, vvl.curVal.get<T>(), desc, false);
    }
    v = std::clamp<T>(rawV, vvl.minVal.get<T>(), vvl.maxVal.get<T>());
    if (rawV != v) {
      RCLCPP_INFO_STREAM(
        node->get_logger(), p.name << " outside limits, adjusted " << rawV << " -> " << v);
      node->set_parameter(rclcpp::Parameter(p.name, v));
    } else {
      RCLCPP_INFO_STREAM(
        node->get_logger(), "parameter " << p.name << " initialized with value " << v);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_WARN_STREAM(
      node->get_logger(), "overwriting bad param with default: " + std::string(e.what()));
    node->declare_parameter(p.name, v, desc, true);
  }
  return (v);
}

Driver::Driver(const rclcpp::NodeOptions & options)
: Node(
    "libcaer_driver",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  messageThresholdTime_ = uint64_t(std::abs(get_or("event_message_time_threshold", 1.0e-3)) * 1e9);
  messageThresholdSize_ =
    static_cast<size_t>(std::abs(get_or("event_message_size_threshold", int64_t(1000000000))));

  const std::string deviceType = get_or("device_type", std::string("davis"));
  try {
    wrapper_.reset(new LibcaerWrapper());
    wrapper_->setCallbackHandler(this);
    wrapper_->initialize(deviceType, get_or("device_id", 1), get_or("serial", std::string()));
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "sensor initialization failed: " << e.what());
    throw(e);
  }
  if (wrapper_->getInfo().hasDVS) {
    eventPub_ = this->create_publisher<EventPacketMsg>(
      "~/events", rclcpp::QoS(rclcpp::KeepLast(get_or("send_queue_size", 1000)))
                    .best_effort()
                    .durability_volatile());
  }
  if (wrapper_->getInfo().hasIMU) {
    imuPub_ = this->create_publisher<ImuMsg>(
      "~/imu", rclcpp::QoS(rclcpp::KeepLast(get_or("imu_send_queue_size", 10)))
                 .best_effort()
                 .durability_volatile());
  }

  wrapper_->initializeParameters(this);

  isMaster_ = get_or<bool>("master", true);
  if (isMaster_) {
    if (!wrapper_->getInfo().deviceIsMaster) {
      RCLCPP_WARN(get_logger(), "this device should be master, but the hardware says it's not!");
    }
    resetPub_ =
      this->create_publisher<TimeMsg>("~/reset_timestamps", rclcpp::QoS(rclcpp::KeepLast(10)));
  } else {
    if (wrapper_->getInfo().deviceIsMaster) {
      RCLCPP_WARN(get_logger(), "this device should be slave, but the hardware says it's not!");
    }

    resetSub_ = this->create_subscription<TimeMsg>(
      "~/reset_timestamps", rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&Driver::resetMsg, this, std::placeholders::_1));
  }

  isBigEndian_ = check_endian::isBigEndian();
  // ------ get other parameters from camera
  cameraFrameId_ = get_or<std::string>("camera_frame_id", "camera");
  imuFrameId_ = get_or<std::string>("imu_frame_id", "imu");
  dvsWidth_ = wrapper_->getInfo().dvsSizeX;
  dvsHeight_ = wrapper_->getInfo().dvsSizeY;
  apsWidth_ = wrapper_->getInfo().apsSizeX;
  apsHeight_ = wrapper_->getInfo().apsSizeY;

  RCLCPP_INFO_STREAM(
    get_logger(), "res: " << dvsWidth_ << " x " << dvsHeight_ << " camera frame: " << cameraFrameId_
                          << " imu frame: " << imuFrameId_);

  infoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, get_name(), get_or<std::string>("camerainfo_url", ""));

  cameraInfoMsg_ = infoManager_->getCameraInfo();
  if (
    ((cameraInfoMsg_.width != 0) && (cameraInfoMsg_.width != apsWidth_)) ||
    ((cameraInfoMsg_.height != 0) && (cameraInfoMsg_.height != apsHeight_))) {
    RCLCPP_WARN(get_logger(), "sensor resolution does not match calibration file!");
  }
  if (cameraInfoMsg_.width == 0) {
    cameraInfoMsg_.width = apsWidth_;
  }
  if (cameraInfoMsg_.height == 0) {
    cameraInfoMsg_.height = apsHeight_;
  }
  cameraInfoMsg_.header.frame_id = cameraFrameId_;

  cameraPub_ =
    image_transport::create_camera_publisher(this, "~/image_raw", rmw_qos_profile_sensor_data);

  start();
}

void Driver::resetMsg(TimeMsg::ConstSharedPtr)
{
  // This message should only be received by the slave
  if (wrapper_) {
    if (wrapper_->getInfo().deviceIsMaster) {
      RCLCPP_WARN(get_logger(), "master received a time reset message, why?");
    }
  }
}

void Driver::resetTime()
{
  if (wrapper_) {
    wrapper_->resetTimeStamps();
  }

  if (isMaster_) {
    TimeMsg msg(this->get_clock()->now());

    resetPub_->publish(msg);
  }
}

Driver::~Driver()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

void Driver::declareParameter(const std::shared_ptr<Parameter> & p, const RosParameter & rp)
{
  switch (rp.type) {
    case RosParameterType::ROS_INTEGER:
      declare_ros_parameter<int>(this, rp);
      break;
    case RosParameterType::ROS_BOOLEAN:
      declare_ros_parameter<bool>(this, rp);
      break;
    default:
      RCLCPP_ERROR_STREAM(get_logger(), "rosparam of unknown type!");
      throw(std::runtime_error("unknown rosparam!"));
      break;
  }
  parameterMap_.insert({rp.name, p});
}

void Driver::updateParameter(
  const std::string & name, std::shared_ptr<Parameter> p, const rclcpp::ParameterValue & rp)
{
  try {
    switch (p->getCaerType()) {
      case CaerParameterType::INTEGER: {
        RCLCPP_INFO_STREAM(get_logger(), "updating int " << name << " to " << rp.get<int>());
        auto ip = std::dynamic_pointer_cast<IntegerParameter>(p);
        auto & vnl = ip->getValueWithLimits();
        const int targetVal =
          std::clamp<int>(rp.get<int>(), vnl.minVal.get<int>(), vnl.maxVal.get<int>());
        vnl.curVal = wrapper_->setIntegerParameter(name, ip, targetVal);
        if (vnl.curVal.get<int>() != rp.get<int>()) {  // send out update to ROS world
          this->set_parameter(rclcpp::Parameter(name, vnl.curVal.get<int>()));
        }
        break;
      }
      case CaerParameterType::BOOLEAN: {
        RCLCPP_INFO_STREAM(
          get_logger(), "updating bool " << name << " to " << static_cast<int>(rp.get<bool>()));
        auto bp = std::dynamic_pointer_cast<BooleanParameter>(p);
        wrapper_->setBooleanParameter(bp, rp.get<bool>());
        break;
      }
      case CaerParameterType::CF_BIAS: {
        auto cfb = std::dynamic_pointer_cast<CoarseFineParameter>(p);
        RCLCPP_INFO_STREAM(get_logger(), "updating bias " << name << " to " << rp.get<int>());
        auto & vnl = cfb->getValueWithLimits(name);
        const int targetBias =
          std::clamp<int>(rp.get<int>(), vnl.minVal.get<int>(), vnl.maxVal.get<int>());
        vnl.curVal = wrapper_->setCourseFineBias(name, cfb, targetBias);
        if (vnl.curVal.get<int>() != rp.get<int>()) {  // send out update to ROS world
          this->set_parameter(rclcpp::Parameter(name, vnl.curVal.get<int>()));
        }
        break;
      }
      case CaerParameterType::VDAC_BIAS: {
        auto vb = std::dynamic_pointer_cast<VDACParameter>(p);
        RCLCPP_INFO_STREAM(get_logger(), "updating vdac bias " << name << " to " << rp.get<int>());
        auto & vnl = vb->getValueWithLimits(name);
        const int targetBias =
          std::clamp<int>(rp.get<int>(), vnl.minVal.get<int>(), vnl.maxVal.get<int>());
        vnl.curVal = wrapper_->setVDACBias(name, vb, targetBias);
        if (vnl.curVal.get<int>() != rp.get<int>()) {  // send out update to ROS world
          this->set_parameter(rclcpp::Parameter(name, vnl.curVal.get<int>()));
        }
        break;
      }

      default:
        // throw(std::runtime_error("invalid parameter type!"));
        break;
    }
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_WARN_STREAM(get_logger(), "ignoring param  " << name << " with invalid type!");
  }
}

void Driver::onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event)
{
  if (event->node != this->get_fully_qualified_name()) {
    return;
  }
  // need to make copy to work around Foxy API
  auto ev = std::make_shared<rcl_interfaces::msg::ParameterEvent>(*event);
  std::vector<std::string> validEvents;
  for (const auto & p : parameterMap_) {
    validEvents.push_back(p.first);
  }
  rclcpp::ParameterEventsFilter filter(
    ev, validEvents, {rclcpp::ParameterEventsFilter::EventType::CHANGED});
  for (auto & ev_it : filter.get_events()) {
    const std::string & name = ev_it.second->name;
    auto it = parameterMap_.find(name);
    if (it != parameterMap_.end()) {
      updateParameter(name, it->second, rclcpp::ParameterValue(ev_it.second->value));
    }
  }
}

void Driver::start()
{
  double printInterval;
  this->get_parameter_or("statistics_print_interval", printInterval, 1.0);
  wrapper_->setStatisticsInterval(printInterval);

  resetTime();

  // ------ start camera, may get callbacks from then on
  wrapper_->startSensor();

  // listen to changes in ROS parameters
  parameterSubscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this, std::bind(&Driver::onParameterEvent, this, std::placeholders::_1));
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
    if (!eventMsg_) {
      eventMsg_.reset(new EventPacketMsg());
      eventMsg_->header.frame_id = cameraFrameId_;
      eventMsg_->encoding = "mono";
      eventMsg_->seq = seq_++;
      eventMsg_->width = dvsWidth_;
      eventMsg_->height = dvsHeight_;
      eventMsg_->is_bigendian = isBigEndian_;
      eventMsg_->events.reserve(reserveSize_);
    }
    (void)message_converter::convert_polarity_packet(
      eventMsg_.get(), packet, rclcpp::Time(t, RCL_SYSTEM_TIME));
    const auto & events = eventMsg_->events;
    if (t - lastMessageTime_ > messageThresholdTime_ || events.size() > messageThresholdSize_) {
      reserveSize_ = std::max(reserveSize_, events.size());
      eventPub_->publish(std::move(eventMsg_));
      lastMessageTime_ = t;
      wrapper_->updateBytesSent(events.size());
      wrapper_->updateMsgsSent(1);
    } else {
      if (!eventMsg_) {
        eventMsg_.reset();
      }
    }
  }
}

void Driver::framePacketCallback(uint64_t t, const libcaer::events::FrameEventPacket & packet)
{
  if (cameraPub_.getNumSubscribers() > 0) {
    std::vector<std::unique_ptr<sensor_msgs::msg::Image>> msgs;
    (void)message_converter::convert_frame_packet(
      &msgs, packet, cameraFrameId_, rclcpp::Time(t, RCL_SYSTEM_TIME));
    for (auto & img : msgs) {
      sensor_msgs::msg::CameraInfo::UniquePtr cinfo(
        new sensor_msgs::msg::CameraInfo(cameraInfoMsg_));
      cameraPub_.publish(std::move(img), std::move(cinfo));
    }
  }
}

void Driver::imu6PacketCallback(uint64_t t, const libcaer::events::IMU6EventPacket & packet)
{
  if (imuPub_->get_subscription_count() > 0) {
    std::vector<std::unique_ptr<sensor_msgs::msg::Imu>> msgs;
    (void)message_converter::convert_imu6_packet(
      &msgs, packet, imuFrameId_, rclcpp::Time(t, RCL_SYSTEM_TIME));
    for (auto & msg : msgs) {
      imuPub_->publish(std::move(msg));
    }
  }
}

}  // namespace libcaer_driver

RCLCPP_COMPONENTS_REGISTER_NODE(libcaer_driver::Driver)
