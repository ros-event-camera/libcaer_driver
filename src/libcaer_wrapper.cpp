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

#include "libcaer_driver/libcaer_wrapper.hpp"

#include <chrono>
#include <libcaercpp/devices/device_discover.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <thread>
#include <tuple>

static std::map<std::string, int> devices = {
  {"dvxplorer", CAER_DEVICE_DVXPLORER}, {"davis", CAER_DEVICE_DAVIS}};

static rclcpp::Logger logger() { return (rclcpp::get_logger("driver")); }

namespace libcaer_driver
{
LibcaerWrapper::LibcaerWrapper() { lastPrintTime_ = std::chrono::system_clock::now(); }

LibcaerWrapper::~LibcaerWrapper()
{
  (void)stopSensor();
  stopThreads();
}

void LibcaerWrapper::initialize(
  const std::string & deviceType, int deviceId, const std::string & serial)
{
  deviceType_ = deviceType;
  deviceId_ = deviceId;
  restrictSN_ = serial;
  initializeSensor();
  startThreads();
}

void LibcaerWrapper::stopThreads()
{
  keepRunning_ = false;
  if (processingThread_) {
    processingThread_->join();
    processingThread_.reset();
  }
  if (statsThread_) {
    statsThread_->join();
    statsThread_.reset();
  }
}

template <class T>
std::unique_ptr<T> open_dev(
  rclcpp::Logger logger, int16_t deviceId, const std::string & restrictSN, std::string * sn)
{
  auto p = std::make_unique<T>(deviceId, 0 /*usb bus*/, 0 /*usb dev*/, restrictSN);
  const auto info = reinterpret_cast<T *>(p.get())->infoGet();
  *sn = info.deviceSerialNumber;
  RCLCPP_INFO(
    logger, "opened %s: id: %d, master: %d, res(%d, %d), logic version: %d", info.deviceString,
    info.deviceID, info.deviceIsMaster, info.dvsSizeX, info.dvsSizeY, info.logicVersion);
  return (p);
}

static std::unique_ptr<libcaer::devices::device> open_device(
  rclcpp::Logger logger, int16_t deviceId, const caer_device_discovery_result & dr,
  const std::string & restrictSN, std::string * sn)
{
  std::unique_ptr<libcaer::devices::device> p;

  switch (dr.deviceType) {
    case CAER_DEVICE_DAVIS:
      p = open_dev<libcaer::devices::davis>(logger, deviceId, restrictSN, sn);
      break;
    case CAER_DEVICE_DVXPLORER:
      p = open_dev<libcaer::devices::dvXplorer>(logger, deviceId, restrictSN, sn);
      break;
    default:
      RCLCPP_ERROR(logger, "found device of unsupported type: %d", dr.deviceType);
      throw(std::runtime_error("unsupported device type!"));
      break;
  }
  return (p);
}

void LibcaerWrapper::initializeSensor()
{
  const auto dev_it = devices.find(deviceType_);
  if (dev_it == devices.end()) {
    throw(std::runtime_error("unsupported device configured: " + deviceType_));
  }
  const auto all_devices = libcaer::devices::discover::all();
  RCLCPP_INFO_STREAM(logger(), "found " << all_devices.size() << " device(s)");

  const int num_tries = 5;
  for (int i = 0; i < num_tries; i++) {
    auto devices = libcaer::devices::discover::device(dev_it->second);
    RCLCPP_INFO_STREAM(
      logger(), "found " << devices.size() << " device(s) of type " << deviceType_ << "("
                         << dev_it->second << ")");
    for (const auto & dev : devices) {
      device_ = open_device(logger(), deviceId_, dev, restrictSN_, &serialNumber_);
      if (device_) {
        break;
      }
    }
    if (!device_) {
      RCLCPP_ERROR_STREAM(
        logger(), "cannot open sensor on attempt " << i + 1 << ", retrying " << num_tries - i - 1
                                                   << " more times");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      break;
    }
  }
  if (!device_) {
    throw(std::runtime_error("failed to open device"));
  }
  device_->sendDefaultConfig();
}

bool LibcaerWrapper::startSensor() { return (true); }

bool LibcaerWrapper::stopSensor() { return (true); }

void LibcaerWrapper::startThreads()
{
  processingThread_ = std::make_shared<std::thread>(&LibcaerWrapper::processingThread, this);
  statsThread_ = std::make_shared<std::thread>(&LibcaerWrapper::statsThread, this);
}

void LibcaerWrapper::processingThread()
{
  const std::chrono::microseconds timeout((int64_t)(1000000LL));
  while (rclcpp::ok() && keepRunning_) {
  }
  RCLCPP_INFO(logger(), "processing thread exited!");
}

void LibcaerWrapper::statsThread()
{
  while (rclcpp::ok() && keepRunning_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(statsInterval_ * 1000)));
    printStatistics();
  }
  RCLCPP_INFO(logger(), "statistics thread exited!");
}

void LibcaerWrapper::printStatistics()
{
  Stats stats;
  {
    std::unique_lock<std::mutex> lock(statsMutex_);
    stats = stats_;
    stats_ = Stats();  // reset statistics
  }
  std::chrono::time_point<std::chrono::system_clock> t_now = std::chrono::system_clock::now();
  const double dt = std::chrono::duration<double>(t_now - lastPrintTime_).count();
  lastPrintTime_ = t_now;
  const double invT = dt > 0 ? 1.0 / dt : 0;
  const double recvByteRate = 1e-6 * stats.bytesRecv * invT;

  const int recvMsgRate = static_cast<int>(stats.msgsRecv * invT);
  const int sendMsgRate = static_cast<int>(stats.msgsSent * invT);

  RCLCPP_INFO(
    logger(),
    "bw in: %9.5f MB/s, msgs/s in: %7d, "
    "out: %7d",
    recvByteRate, recvMsgRate, sendMsgRate);
}

}  // namespace libcaer_driver
