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
  encoding_ = get_or<std::string>("encoding", "libcaer_cmp");
  useCompressed_ = (encoding_ == "libcaer_cmp");
  if (encoding_ != "libcaer_cmp" && encoding_ != "libcaer") {
    BOMB_OUT("invalid encoding: " << encoding_);
  }
  const std::string deviceType = get_or("device_type", std::string("davis"));
  try {
    wrapper_.reset(new LibcaerWrapper());
    wrapper_->setCallbackHandler(this);
    wrapper_->initialize(deviceType, get_or("device_id", 1), get_or("serial", std::string()));
  } catch (std::runtime_error & e) {
    BOMB_OUT("sensor initialization failed: " << e.what());
  }
  if (wrapper_->hasDVS()) {
    eventPub_ = this->create_publisher<EventPacketMsg>(
      "~/events", rclcpp::QoS(rclcpp::KeepLast(get_or("event_send_queue_size", 1000)))
                    .best_effort()
                    .durability_volatile());
  }
  if (wrapper_->hasIMU()) {
    imuPub_ = this->create_publisher<ImuMsg>(
      "~/imu", rclcpp::QoS(rclcpp::KeepLast(get_or("imu_send_queue_size", 10)))
                 .best_effort()
                 .durability_volatile());
  }
#ifdef USE_PUB_THREAD
  keepPubThreadRunning_.store(true);
  pubThread_ = std::make_shared<std::thread>(&Driver::publishingThread, this);
#endif

  wrapper_->initializeParameters(this);
  autoExposureEnabled_ = get_or<bool>("auto_exposure_enabled", false);
  LOG_INFO("auto exposure enabled: " << (autoExposureEnabled_ ? "True" : "False"));
  parameterMap_.insert(
    {"auto_exposure_illumination", declareRosParameter(std::make_shared<RosIntParameter>(
                                     "auto_exposure_illumination", 127, 0, 255,
                                     "auto exposure target illumination", nullptr, FIELD_INT))});
  targetIllumination_ = get_parameter("auto_exposure_illumination").as_int();
  parameterMap_.insert(
    {"auto_exposure_hysteresis", declareRosParameter(std::make_shared<RosFloatParameter>(
                                   "auto_exposure_hysteresis", 0.0625, 0, 0.5,
                                   "auto exposure hysteresis", nullptr, FIELD_FLOAT))});
  exposureHysteresis_ = get_parameter("auto_exposure_hysteresis").as_double();

  isMaster_ = get_or<bool>("master", true);
  if (isMaster_) {
    if (!wrapper_->isMaster()) {
      LOG_WARN("this device should be master, but the hardware says it's not!");
    }
    resetPub_ =
      this->create_publisher<TimeMsg>("~/reset_timestamps", rclcpp::QoS(rclcpp::KeepLast(10)));
  } else {
    if (wrapper_->isMaster()) {
      LOG_WARN("this device is configured as slave, but the hardware says it's not!");
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

  LOG_INFO_FMT(
    "res: %d x %d,  camera frame id : %s, imu frame id: %s", dvsWidth_, dvsHeight_,
    cameraFrameId_.c_str(), imuFrameId_.c_str());

  infoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, get_name(), get_or<std::string>("camerainfo_url", ""));

  cameraInfoMsg_ = infoManager_->getCameraInfo();
  if (
    ((cameraInfoMsg_.width != 0) && (cameraInfoMsg_.width != apsWidth_)) ||
    ((cameraInfoMsg_.height != 0) && (cameraInfoMsg_.height != apsHeight_))) {
    LOG_WARN("sensor resolution does not match calibration file!");
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

void Driver::resetMsg(TimeMsg::ConstSharedPtr msg)
{
  // This message should only be received by the slave
  if (wrapper_) {
    if (wrapper_->isMaster()) {
      LOG_WARN("master received a time reset message, why?");
    } else {
      rosBaseTime_ = *msg;
    }
  }
}

void Driver::resetTime()
{
  if (wrapper_) {
    wrapper_->resetTimeStamps();
  }

  if (isMaster_) {
    // round the time to nearest microsecond so the event time stamps
    // are prettier
    rosBaseTime_ =
      rclcpp::Time((this->get_clock()->now().nanoseconds() / 1000ULL) * 1000ULL, RCL_SYSTEM_TIME);
    TimeMsg msg(rosBaseTime_);
    resetPub_->publish(msg);
  }
}

Driver::~Driver()
{
  stop();
  wrapper_.reset();  // invoke destructor
#ifdef USE_PUB_THREAD
  if (pubThread_) {
    {
      keepPubThreadRunning_.store(false);
      std::unique_lock<std::mutex> lock(pubMutex_);
      pubCv_.notify_all();
    }
    pubThread_->join();
    pubThread_.reset();
  }
#endif
}

std::shared_ptr<RosIntParameter> Driver::declareRosParameter(
  const std::shared_ptr<RosIntParameter> & rp)
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
    try {
      (void)get_parameter_or<int32_t>(name, vRos, rp->getValue());
    } catch (const rclcpp::ParameterTypeException & e) {
      LOG_WARN("ignoring param " << name << " with invalid type!");
    }
    // first undeclare it so we can redeclare it with the right type
    if (this->has_parameter(name)) {
      this->undeclare_parameter(name);
    }
    vRos = this->declare_parameter(name, vRos, desc, true);
    const int32_t vClamped = rp->clamp(vRos);
    rp->getParameter()->setValue(
      rp->getField(), vClamped);  // libcaer_wrapper will use this for initialization
    if (vClamped != vRos) {
      LOG_INFO(name << " is outside limits, adjusted " << vRos << " -> " << vClamped);
      this->set_parameter(rclcpp::Parameter(name, vClamped));
    } else {
      LOG_INFO_FMT("%-25s set to: %5d", name.c_str(), vRos);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    LOG_WARN("overwriting bad param with default: " + std::string(e.what()));
    this->undeclare_parameter(name);
    this->declare_parameter(name, vRos, desc, true);
  }
  return (rp);
}

std::shared_ptr<RosFloatParameter> Driver::declareRosParameter(
  const std::shared_ptr<RosFloatParameter> & rp)
{
  const std::string & name = rp->getName();
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = name;
  desc.description = rp->getDescription();
  rcl_interfaces::msg::FloatingPointRange fr;
  fr.from_value = rp->getMinValue();
  fr.to_value = rp->getMaxValue();
  fr.step = 0;
  desc.floating_point_range.push_back(fr);
  float vRos(rp->getValue());
  try {
    // declare or get the parameters value if it's already declared
    try {
      (void)get_parameter_or<float>(name, vRos, rp->getValue());
    } catch (const rclcpp::ParameterTypeException & e) {
      LOG_WARN("ignoring param " << name << " with invalid type!");
    }
    // first undeclare it so we can redeclare it with the right type
    if (this->has_parameter(name)) {
      this->undeclare_parameter(name);
    }
    vRos = this->declare_parameter(name, vRos, desc, true);
    const auto vClamped = rp->clamp(vRos);
    auto p = rp->getParameter();
    if (p) {
      // libcaer_wrapper will use this for initialization
      p->setValue(rp->getField(), vClamped);
    }
    if (vClamped != vRos) {
      LOG_INFO(name << " is outside limits, adjusted " << vRos << " -> " << vClamped);
      this->set_parameter(rclcpp::Parameter(name, vClamped));
    } else {
      LOG_INFO_FMT("%-25s set to: %10.5f", name.c_str(), vRos);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    LOG_WARN("overwriting bad param with default: " + std::string(e.what()));
    this->undeclare_parameter(name);
    this->declare_parameter(name, vRos, desc, true);
  }
  return (rp);
}

std::shared_ptr<RosBoolParameter> Driver::declareRosParameter(
  const std::shared_ptr<RosBoolParameter> & rp)
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
    LOG_INFO_FMT("%-25s set to: %5s", name.c_str(), (p->getValue() ? "True" : "False"));
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    LOG_WARN("overwriting bad param with default: " + std::string(e.what()));
    this->declare_parameter(name, p->getValue(), desc, true);
  }
  return (rp);
}

void Driver::declareParameterCallback(const std::shared_ptr<RosParameter> & rp)
{
  switch (rp->getType()) {
    case ROS_INT: {
      declareRosParameter(std::dynamic_pointer_cast<RosIntParameter>(rp));
      if (rp->getName() == "aps_exposure") {
        exposureParameter_ = std::dynamic_pointer_cast<RosIntParameter>(rp);
      }
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

void Driver::updateParameter(std::shared_ptr<RosParameter> rp, const rclcpp::ParameterValue & vArg)
{
  rclcpp::ParameterValue v(vArg);
  const auto & name = rp->getName();
  auto p = rp->getParameter();
  if (!p) {
    updateDriverParameter(rp, vArg);
    return;
  }
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
    LOG_WARN("ignoring param  " << name << " with invalid type!");
  }
}

void Driver::updateDriverParameter(
  std::shared_ptr<RosParameter> rp, const rclcpp::ParameterValue & vArg)
{
  if (rp->getName() == "auto_exposure_enabled") {
    autoExposureEnabled_ = vArg.get<bool>();
    LOG_INFO("auto exposure enabled: " << (autoExposureEnabled_ ? "True" : "False"));
  } else if (rp->getName() == "auto_exposure_illumination") {
    targetIllumination_ = static_cast<int32_t>(vArg.get<int>());
    LOG_INFO("target illumination set to: " << targetIllumination_);
  } else if (rp->getName() == "auto_exposure_hysteresis") {
    exposureHysteresis_ = static_cast<float>(vArg.get<double>());
    LOG_INFO("auto exposure hysteresis set to: " << exposureHysteresis_);
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
  LOG_INFO("driver stopping sensor");
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
      eventMsg_->header.stamp = rclcpp::Time(t, RCL_SYSTEM_TIME);
      eventMsg_->encoding = encoding_;
      eventMsg_->seq = seq_++;
      eventMsg_->width = dvsWidth_;
      eventMsg_->height = dvsHeight_;
      eventMsg_->is_bigendian = isBigEndian_;
      eventMsg_->events.reserve(reserveSize_);
    }
    if (useCompressed_) {
      message_converter::convert_polarity_packet_compressed(
        eventMsg_.get(), packet, rosBaseTime_, &sensorTime_0);
    } else {
      message_converter::convert_polarity_packet(eventMsg_.get(), packet, rosBaseTime_);
    }

    const auto & events = eventMsg_->events;
    if (t - lastMessageTime_ > messageThresholdTime_ || events.size() > messageThresholdSize_) {
      reserveSize_ = std::max(reserveSize_, events.size());
#ifdef USE_PUB_THREAD
      {
        std::unique_lock<std::mutex> lock(pubMutex_);
        pubQueue_.push(std::move(eventMsg_));
        pubCv_.notify_all();
      }
#else
      eventPub_->publish(std::move(eventMsg_));  // will reset the pointer
#endif
      lastMessageTime_ = t;
      wrapper_->updateBytesSent(events.size());
      wrapper_->updateMsgsSent(1);
    }
  } else {
    if (eventMsg_) {
      eventMsg_.reset();
    }
  }
}

static int32_t compute_new_exposure_time(
  const sensor_msgs::msg::Image & img, float * illum, int32_t currentTime, int32_t targetIllum)
{
  const auto & im = img.data;
  *illum = static_cast<float>(std::accumulate(im.begin(), im.end(), 0)) / im.size();
  if (*illum == 0.0 || targetIllum / *illum > 1e3) {
    return (std::numeric_limits<int32_t>::max());
  }
  const float alpha = 0.8;
  const int32_t newTime = currentTime * pow(targetIllum / *illum, alpha);
  return (newTime);
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
      const int32_t currentTime = wrapper_->getExposureTime();
      if (autoExposureEnabled_ && exposureParameter_) {
        if (frameDelay_ == 0) {
          float illum;
          int32_t newTime =
            compute_new_exposure_time(*img, &illum, currentTime, targetIllumination_);
          newTime = exposureParameter_->clamp(newTime);
          if (abs(newTime - currentTime) > static_cast<int>(currentTime * exposureHysteresis_)) {
            LOG_INFO_FMT(
              "autoexp: illum: %12.5f, exp time: %5d -> %5d", illum, currentTime, newTime);
            wrapper_->setExposureTime(newTime);
            this->set_parameter(rclcpp::Parameter("aps_exposure", newTime));
            frameDelay_ = 1;
          }
        } else {
          frameDelay_--;
        }
      }
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

#ifdef USE_PUB_THREAD
void Driver::publishingThread()
{
  const auto duration = std::chrono::milliseconds(100);
  while (rclcpp::ok() && keepPubThreadRunning_.load()) {
    std::unique_lock<std::mutex> lock(pubMutex_);
    pubCv_.wait_for(lock, duration);
    while (!pubQueue_.empty()) {
      eventPub_->publish(std::move(pubQueue_.front()));
      pubQueue_.pop();
    }
  }
  LOG_INFO("publishing thread exited!");
}
#endif

}  // namespace libcaer_driver

RCLCPP_COMPONENTS_REGISTER_NODE(libcaer_driver::Driver)
