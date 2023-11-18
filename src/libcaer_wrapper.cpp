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
#include <libcaer_driver/action.hpp>
#include <libcaer_driver/libcaer_wrapper.hpp>
#include <libcaer_driver/resize_hack.hpp>
#include <libcaercpp/devices/device_discover.hpp>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <thread>
#include <tuple>

namespace libcaer_driver
{
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::system_clock;
//
// ------------- local static functions ------------
static const std::map<std::string, int> devices = {
  {"dvxplorer", CAER_DEVICE_DVXPLORER}, {"davis", CAER_DEVICE_DAVIS}};

// local logger handle
static rclcpp::Logger logger() { return (rclcpp::get_logger("driver")); }

// need to use free function for usb disconnect handling
static void device_disconnected(void * ptr)
{
  reinterpret_cast<LibcaerWrapper *>(ptr)->deviceDisconnected();
}

// some functions to work around silly template restrictions

template <>
int32_t detail::set_parameter(
  libcaer_driver::LibcaerWrapper * wrapper, const std::string & name, const Parameter & p,
  int32_t value)
{
  return (wrapper->setIntegerParameter(name, p, value));
}

template <>
bool detail::set_parameter(
  libcaer_driver::LibcaerWrapper * wrapper, const std::string & name, const Parameter & p,
  bool value)
{
  return (static_cast<bool>(wrapper->setIntegerParameter(name, p, static_cast<bool>(value))));
}

LibcaerWrapper::LibcaerWrapper()
{
  lastPrintTime_ = system_clock::now();
  keepProcessingRunning_.store(false);
  keepStatsRunning_.store(true);
  statsThread_ = std::make_shared<std::thread>(&LibcaerWrapper::statsThread, this);
}

LibcaerWrapper::~LibcaerWrapper()
{
  stopSensor();
  stopStatsThread();
  device_.reset();
}

void LibcaerWrapper::deviceDisconnected()
{
  RCLCPP_ERROR(logger(), "device disconnected!");
  stopSensor();
  stopStatsThread();
  throw(std::runtime_error("device disconnected!"));
}

void LibcaerWrapper::stopStatsThread()
{
  if (statsThread_) {
    {
      keepStatsRunning_.store(false);
      std::unique_lock<std::mutex> lock(statsMutex_);
      statsCv_.notify_all();
    }
    statsThread_->join();
    statsThread_.reset();
  }
}

void LibcaerWrapper::stopProcessingThread()
{
  keepProcessingRunning_.store(false);
  if (processingThread_) {
    processingThread_->join();
    processingThread_.reset();
  }
}

template <class T>
std::unique_ptr<T> open_dev(
  rclcpp::Logger logger, int16_t deviceId, const std::string & restrictSN, int * width,
  int * height, std::string * sn)
{
  auto p = std::make_unique<T>(deviceId, 0 /*usb bus*/, 0 /*usb dev*/, restrictSN);
  const auto info = reinterpret_cast<T *>(p.get())->infoGet();
  *sn = info.deviceSerialNumber;
  *width = info.dvsSizeX;
  *height = info.dvsSizeY;
  RCLCPP_INFO(
    logger, "opened %s: id: %d, master: %d, res(%d, %d), logic version: %d", info.deviceString,
    info.deviceID, info.deviceIsMaster, info.dvsSizeX, info.dvsSizeY, info.logicVersion);
  return (p);
}

static std::unique_ptr<libcaer::devices::device> open_device(
  rclcpp::Logger logger, int16_t deviceId, const caer_device_discovery_result & dr,
  const std::string & restrictSN, int * width, int * height, std::string * sn)
{
  std::unique_ptr<libcaer::devices::device> p;

  switch (dr.deviceType) {
    case CAER_DEVICE_DAVIS:
      p = open_dev<libcaer::devices::davis>(logger, deviceId, restrictSN, width, height, sn);
      break;
    case CAER_DEVICE_DVXPLORER:
      p = open_dev<libcaer::devices::dvXplorer>(logger, deviceId, restrictSN, width, height, sn);
      break;
    default:
      RCLCPP_ERROR(logger, "found device of unsupported type: %d", dr.deviceType);
      throw(std::runtime_error("unsupported device type!"));
      break;
  }
  return (p);
}

static void log_all_devices()
{
  const auto all_devices = libcaer::devices::discover::all();
  if (all_devices.empty()) {
    RCLCPP_ERROR(logger(), "found no devices!");
  }
  RCLCPP_INFO_STREAM(logger(), "found devices: " << all_devices.size());
  for (const auto & dev : all_devices) {
    std::string devInfo("unknown device type");
    switch (dev.deviceType) {
      case CAER_DEVICE_DAVIS:
        devInfo = "DAVIS SN: " + std::string(dev.deviceInfo.davisInfo.deviceSerialNumber);
        break;
      case CAER_DEVICE_DVXPLORER:
        devInfo = "DVXPLORER SN: " + std::string(dev.deviceInfo.dvXplorerInfo.deviceSerialNumber);
        break;
      default:
        break;
    }
    RCLCPP_INFO(logger(), "found device: %-30s", devInfo.c_str());
  }
}

void LibcaerWrapper::initialize(
  const std::string & devType, int deviceId, const std::string & restrictSN)
{
  log_all_devices();
  const auto dev_it = devices.find(devType);
  if (dev_it == devices.end()) {
    throw(std::runtime_error("unsupported device configured: " + devType));
  }
  deviceType_ = dev_it->second;

  const int num_tries = 5;
  for (int i = 0; i < num_tries; i++) {
    auto devices = libcaer::devices::discover::device(deviceType_);
    for (const auto & dev : devices) {
      device_ = open_device(logger(), deviceId, dev, restrictSN, &width_, &height_, &serialNumber_);
      if (device_) {
        break;
      }
    }
    if (!device_) {
      RCLCPP_ERROR_STREAM(
        logger(), "cannot open sensor on attempt " << i + 1 << ", retrying " << num_tries - i - 1
                                                   << " more times");
      std::this_thread::sleep_for(nanoseconds(1000000000));
    } else {
      break;
    }
  }
  if (!device_) {
    throw(std::runtime_error("failed to open device"));
  }
  device_->sendDefaultConfig();
  device_->configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
}

bool LibcaerWrapper::startSensor()
{
  if (!keepProcessingRunning_.load()) {
    keepProcessingRunning_.store(true);
    // must start the device *before* the processing thread
    device_->dataStart(nullptr, nullptr, nullptr, device_disconnected, this);
    deviceRunning_ = true;
    startProcessingThread();
    return (true);
  }
  return (false);
}

void LibcaerWrapper::stopSensor()
{
  stopProcessingThread();
  if (deviceRunning_) {
    device_->dataStop();
    deviceRunning_ = false;
    RCLCPP_INFO(logger(), "stopped sensor.");
  }
}

void LibcaerWrapper::startProcessingThread()
{
  processingThread_ = std::make_shared<std::thread>(&LibcaerWrapper::processingThread, this);
}

void LibcaerWrapper::processPacket(
  uint64_t nsSinceEpoch, const libcaer::events::EventPacket & packet)
{
  const auto N = packet.getEventNumber();
  if (N == 0) {
    return;
  }
  switch (packet.getEventType()) {
    case POLARITY_EVENT: {
      const auto & ppacket = static_cast<const libcaer::events::PolarityEventPacket &>(packet);
      callbackHandler_->polarityPacketCallback(nsSinceEpoch, ppacket);
      {
        std::unique_lock<std::mutex> lock(statsMutex_);
        stats_.bytesRecv += packet.getEventNumber() * sizeof(libcaer::events::PolarityEvent);
        stats_.msgsRecv++;
        stats_.eventsRecv += ppacket.getEventNumber();
      }
      break;
    }
    case FRAME_EVENT: {
      const auto & fpacket = static_cast<const libcaer::events::FrameEventPacket &>(packet);
      callbackHandler_->framePacketCallback(nsSinceEpoch, fpacket);
      break;
    }
    case IMU6_EVENT: {
      const auto & fpacket = static_cast<const libcaer::events::IMU6EventPacket &>(packet);
      callbackHandler_->imu6PacketCallback(nsSinceEpoch, fpacket);
      break;
    }
    default:
      break;
  }
}

void LibcaerWrapper::processingThread()
{
  RCLCPP_INFO(logger(), "starting processing thread!");

  while (rclcpp::ok() && keepProcessingRunning_.load(std::memory_order_relaxed)) {
    std::unique_ptr<libcaer::events::EventPacketContainer> pcp = device_->dataGet();
    if (pcp) {
      const uint64_t nsSinceEpoch =
        duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
      for (const auto & packet : *pcp) {
        if (packet) {
          processPacket(nsSinceEpoch, *packet);
        }
      }
    }
  }
  RCLCPP_INFO(logger(), "processing thread exited!");
}

void LibcaerWrapper::statsThread()
{
  const auto duration = nanoseconds(static_cast<int>(statsInterval_ * 1000000000));
  while (rclcpp::ok() && keepStatsRunning_.load()) {
    std::unique_lock<std::mutex> lock(statsMutex_);
    statsCv_.wait_for(lock, duration);
    printStatistics();
  }
  RCLCPP_INFO(logger(), "statistics thread exited!");
}

static uint8_t & pick(struct caer_bias_coarsefine & cfb, const std::string & name)
{
  if (name.find("_fine") != std::string::npos) {
    return (cfb.fineValue);
  } else if (name.find("_coarse") != std::string::npos) {
    return (cfb.coarseValue);
  }
  RCLCPP_ERROR_STREAM(logger(), "bias has no coarse/fine: " << name);
  throw(std::runtime_error("bias has no coarse/fine"));
}

int32_t LibcaerWrapper::setIntegerParameter(
  const std::string & name, const Parameter & p, int32_t value)
{
  int32_t actualValue = value;
  if (p.modAddr == DAVIS_CONFIG_BIAS) {
    actualValue = setCoarseFineBias(name, p, value);
  } else {
    try {
      device_->configSet(p.modAddr, p.paramAddr, value);
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN_STREAM(logger(), "failed to set configuration for " << name << " " << e.what());
    }
    if (p.sexN) {
      try {
        actualValue = device_->configGet(p.modAddr, p.paramAddr);
      } catch (const std::runtime_error & e) {
        // configGet() fails on e.g. dvXplorer
        // RCLCPP_WARN_STREAM(logger(), "could not read back " << name << " " << e.what());
      }
    }
  }
  return (actualValue);
}

int LibcaerWrapper::setCoarseFineBias(const std::string & name, const Parameter & p, int value)
{
  // first read the current bias to get either coarse or fine value, whichever
  // is not being set right now
  const auto modAddr = p.modAddr;
  auto newBias = caerBiasCoarseFineParse(device_->configGet(modAddr, p.paramAddr));
  const int prevValue = pick(newBias, name);
  pick(newBias, name) = value;  // this sets the new value!
  newBias.enabled = true;
  newBias.sexN = p.sexN;
  newBias.typeNormal = true;
  newBias.currentLevelNormal = true;
  device_->configSet(modAddr, p.paramAddr, caerBiasCoarseFineGenerate(newBias));
  // read back one last time
  newBias = caerBiasCoarseFineParse(device_->configGet(modAddr, p.paramAddr));
  const int newValue = pick(newBias, name);
  if (prevValue != newValue) {
    RCLCPP_INFO_STREAM(logger(), name << " changed from " << prevValue << " to " << newValue);
  } else {
    RCLCPP_INFO_STREAM(logger(), name << " left unchanged at " << prevValue);
  }
  return (pick(newBias, name));
}

const ParameterMap & LibcaerWrapper::getParameters()
{
  try {
    return (Parameter::getMap(deviceType_));
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger(), "no parameters defined for device " << deviceType_);
    throw(e);
  }
}

void LibcaerWrapper::resetTimeStamps()
{
  const auto a = action::get(deviceType_, "time_reset");
  device_->configSet(std::get<0>(a), std::get<1>(a), std::get<2>(a));
}

void LibcaerWrapper::printStatistics()
{
  std::chrono::time_point<system_clock> t_now = system_clock::now();
  const double dt = std::chrono::duration<double>(t_now - lastPrintTime_).count();
  lastPrintTime_ = t_now;
  const double invT = dt > 0 ? 1.0 / dt : 0;
  const double recvByteRate = 1e-6 * stats_.bytesRecv * invT;
  const double recvEventsRate = 1e-6 * stats_.eventsRecv * invT;

  const int recvMsgRate = static_cast<int>(stats_.msgsRecv * invT);
  const int sendMsgRate = static_cast<int>(stats_.msgsSent * invT);

  RCLCPP_INFO(
    logger(), "in: %9.5f Mev/s, %8.3f MB/s, %5d msgs/s, out: %5d msg/s", recvEventsRate,
    recvByteRate, recvMsgRate, sendMsgRate);
  stats_ = Stats();  // reset statistics
}

}  // namespace libcaer_driver
