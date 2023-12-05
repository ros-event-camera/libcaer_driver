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
#include <libcaer_driver/logging.hpp>
#include <libcaer_driver/message_converter.hpp>
#include <map>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

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

  const std::string deviceType = get_or("device_type", std::string("davis"));
  try {
    wrapper_.reset(new LibcaerWrapper());
    wrapper_->setCallbackHandler(this);
    wrapper_->initialize(deviceType, get_or("device_id", 1), get_or("serial", std::string()));
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "sensor initialization failed: " << e.what());
    throw(e);
  }
  if (wrapper_->hasDVS()) {
    eventPub_ = this->create_publisher<EventPacketMsg>(
      "~/events", rclcpp::QoS(rclcpp::KeepLast(get_or("send_queue_size", 1000)))
                    .best_effort()
                    .durability_volatile());
  }
  if (wrapper_->hasIMU()) {
    imuPub_ = this->create_publisher<ImuMsg>(
      "~/imu", rclcpp::QoS(rclcpp::KeepLast(get_or("imu_send_queue_size", 10)))
                 .best_effort()
                 .durability_volatile());
  }

  wrapper_->initializeParameters(this);

  isMaster_ = get_or<bool>("master", true);
  if (isMaster_) {
    if (!wrapper_->isMaster()) {
      RCLCPP_WARN(get_logger(), "this device should be master, but the hardware says it's not!");
    }
    resetPub_ =
      this->create_publisher<TimeMsg>("~/reset_timestamps", rclcpp::QoS(rclcpp::KeepLast(10)));
  } else {
    if (wrapper_->isMaster()) {
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
  dvsWidth_ = wrapper_->getDVSSizeX();
  dvsHeight_ = wrapper_->getDVSSizeY();
  apsWidth_ = wrapper_->getAPSSizeX();
  apsHeight_ = wrapper_->getAPSSizeY();

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
    if (wrapper_->isMaster()) {
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

void Driver::declareRosParameter(const std::shared_ptr<RosIntParameter> & rp)
{
  const std::string & name = rp->getName();
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = name;
  desc.description = rp->getDescription();
  rcl_interfaces::msg::IntegerRange ir;
  ir.from_value = rp->getMinValue();
  ir.to_value = rp->getMaxValue();
  ir.step = 1;
  desc.integer_range.push_back(ir);

  int32_t vRos(rp->getValue());
  try {
    // declare or get the parameters value if it's already declared
    if (this->has_parameter(name)) {
      try {
        vRos = this->get_parameter(name).get_value<int>();  // get user-provided ROS value
      } catch (const rclcpp::ParameterTypeException & e) {
        LOG_WARN("ignoring param " << name << " with invalid type!");
      }
    } else {
      vRos = this->declare_parameter(name, vRos, desc, false);
    }
    const int32_t vClamped = rp->clamp(vRos);
    rp->getParameter()->setValue(
      rp->getField(), vClamped);  // libcaer_wrapper will use this for initialization
    if (vClamped != vRos) {
      LOG_INFO(name << " is outside limits, adjusted " << vRos << " -> " << vClamped);
      this->set_parameter(rclcpp::Parameter(name, vClamped));
    } else {
      LOG_INFO("parameter " << name << " initialized with value " << vRos);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    LOG_WARN("overwriting bad param with default: " + std::string(e.what()));
    this->declare_parameter(name, vRos, desc, true);
  }
}

void Driver::declareRosParameter(const std::shared_ptr<RosBoolParameter> & rp)
{
  const std::string & name = rp->getName();
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = name;
  desc.description = rp->getDescription();
  auto p = std::dynamic_pointer_cast<BooleanParameter>(rp->getParameter());
  try {
    if (this->has_parameter(name)) {
      try {
        p->setValue(this->get_parameter(name).get_value<bool>());  // get user-provided ROS value
      } catch (const rclcpp::ParameterTypeException & e) {
        LOG_WARN("ignoring param " << name << " with invalid type!");
      }
    } else {
      p->setValue(this->declare_parameter(name, p->getValue(), desc, false));
    }
    LOG_INFO(
      "parameter " << name << " initialized with value " << (p->getValue() ? "True" : "False"));
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    LOG_WARN("overwriting bad param with default: " + std::string(e.what()));
    this->declare_parameter(name, p->getValue(), desc, true);
  }
}

void Driver::declareParameterCallback(const std::shared_ptr<RosParameter> & rp)
{
  switch (rp->getType()) {
    case ROS_INT: {
      const auto foo = std::dynamic_pointer_cast<RosIntParameter>(rp);
      declareRosParameter(foo);
      break;
    }
    case RosParameterType::ROS_BOOL:
      declareRosParameter(std::dynamic_pointer_cast<RosBoolParameter>(rp));
      break;
    default:
      BOMB_OUT("rosparam of unknown type: " << static_cast<int>(rp->getType()));
      break;
  }
  parameterMap_.insert({rp->getName(), rp});
}

void Driver::deviceDisconnectedCallback()
{
  if (wrapper_) {
    wrapper_->deviceDisconnected();
  }
  throw(std::runtime_error("device disconnected!"));
}

void Driver::updateParameter(std::shared_ptr<RosParameter> rp, const rclcpp::ParameterValue & v)
{
  const auto & name = rp->getName();
  auto p = rp->getParameter();
  try {
    switch (p->getCaerType()) {
      case CaerParameterType::INTEGER: {
        LOG_INFO("updating int " << name << " to " << v.get<int>());
        auto ip = std::dynamic_pointer_cast<IntegerParameter>(p);
        auto rip = std::dynamic_pointer_cast<RosIntParameter>(rp);
        const int32_t targetVal = rip->clamp(v.get<int>());
        ip->setValue(targetVal);
        wrapper_->setIntegerParameter(ip);
        if (ip->getValue() != v.get<int>()) {  // send out update to ROS world
          this->set_parameter(rclcpp::Parameter(name, ip->getValue()));
        }
        break;
      }
      case CaerParameterType::BOOLEAN: {
        LOG_INFO("updating bool " << name << " to " << (v.get<bool>() ? "True" : "False"));
        auto bp = std::dynamic_pointer_cast<BooleanParameter>(p);
        bp->setValue(v.get<bool>());
        wrapper_->setBooleanParameter(bp);
        if (bp->getValue() != v.get<bool>()) {  // send out update to ROS world
          this->set_parameter(rclcpp::Parameter(name, bp->getValue()));
        }
        break;
      }
      case CaerParameterType::CF_BIAS: {
        auto cfb = std::dynamic_pointer_cast<CoarseFineParameter>(p);
        LOG_INFO("updating coarse-fine bias " << name << " to " << v.get<int>());
        auto rip = std::dynamic_pointer_cast<RosIntParameter>(rp);
        const int32_t targetVal = rip->clamp(v.get<int>());
        cfb->setBias(rp->getField(), targetVal);  // sets coarse or fine
        wrapper_->setCoarseFineBias(cfb);         // will update cfb with read-back value
        const int32_t actualVal = cfb->getValue(rp->getField());  // check read back
        if (actualVal != v.get<int>()) {                          // send out update to ROS world
          this->set_parameter(rclcpp::Parameter(name, actualVal));
        }
        break;
      }
      case CaerParameterType::VDAC_BIAS: {
        auto vb = std::dynamic_pointer_cast<VDACParameter>(p);
        LOG_INFO("updating vdac bias " << name << " to " << v.get<int>());
        auto rip = std::dynamic_pointer_cast<RosIntParameter>(rp);
        const int32_t targetVal = rip->clamp(v.get<int>());
        vb->setBias(rp->getField(), targetVal);
        const int32_t actualVal = vb->getValue(rp->getField());  // check read back
        if (actualVal != v.get<int>()) {                         // send out update to ROS world
          this->set_parameter(rclcpp::Parameter(name, actualVal));
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
      updateParameter(it->second, rclcpp::ParameterValue(ev_it.second->value));
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
