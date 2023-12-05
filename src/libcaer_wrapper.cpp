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
#include <libcaer_driver/libcaer_wrapper.hpp>
#include <libcaer_driver/logging.hpp>
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
// local logger handle
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("libcaer_wrapper")); }

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
  LOG_ERROR("device disconnected!");
  stopSensor();
  stopStatsThread();
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
void copy_common_fields(DeviceInfo * out, const T & info)
{
  out->deviceID = info.deviceID;
  out->deviceSerialNumber = info.deviceSerialNumber;
  out->deviceUSBBusNumber = info.deviceUSBBusNumber;
  out->deviceUSBDeviceAddress = info.deviceUSBDeviceAddress;
  out->deviceString = info.deviceString;
  out->firmwareVersion = info.firmwareVersion;
  out->logicVersion = info.logicVersion;
  out->chipID = info.chipID;
  out->deviceIsMaster = info.deviceIsMaster;
  out->muxHasStatistics = info.muxHasStatistics;
  out->dvsSizeX = info.dvsSizeX;
  out->dvsSizeY = info.dvsSizeY;
  out->dvsHasStatistics = info.dvsHasStatistics;
  out->imuType = info.imuType;
  out->extInputHasGenerator = info.extInputHasGenerator;
}

template <class T>
void copy_specific_fields(DeviceInfo *, const T &)
{
  // nothing specific to copy (dvXplorer)
}

// specialize template for davis
template <>
void copy_specific_fields(DeviceInfo * out, const caer_davis_info & info)
{
  out->dvsHasPixelFilter = info.dvsHasPixelFilter;
  out->dvsHasBackgroundActivityFilter = info.dvsHasBackgroundActivityFilter;
  out->dvsHasROIFilter = info.dvsHasROIFilter;
  out->dvsHasSkipFilter = info.dvsHasSkipFilter;
  out->dvsHasPolarityFilter = info.dvsHasPolarityFilter;
  out->apsSizeX = info.apsSizeX;
  out->apsSizeY = info.apsSizeY;
  out->apsColorFilter = info.apsColorFilter;
  out->apsHasGlobalShutter = info.apsHasGlobalShutter;
}

void LibcaerWrapper::initialize(
  const std::string & devType, int deviceId, const std::string & restrictSN)
{
  const auto devices = Device::logAllDevices();
  bool foundDevice = false;
  for (const auto & dev : devices) {
    if ((dev.first == devType) && (restrictSN.empty() || restrictSN == dev.second)) {
      foundDevice = true;
    }
  }
  if (!foundDevice) {
    BOMB_OUT(
      "cannot find device with matching type (" << devType << ") and serial number " << restrictSN);
  }
  const int num_tries = 5;
  for (int i = 0; i < num_tries; i++) {
    device_ = Device::newInstance(devType, deviceId, restrictSN);
    if (!device_) {
      LOG_ERROR(
        "cannot open device of type " << devType << " on attempt " << i + 1 << ", retrying "
                                      << num_tries - i - 1 << " more times");
      std::this_thread::sleep_for(nanoseconds(1000000000));
    } else {
      break;
    }
  }
  if (!device_) {
    BOMB_OUT("failed to open device");
  }
  LOG_INFO("device opened successfully!");
}

bool LibcaerWrapper::startSensor()
{
  if (!keepProcessingRunning_.load()) {
    keepProcessingRunning_.store(true);
    // must start the device *before* the processing thread
    device_->start(callbackHandler_);
    startProcessingThread();
    return (true);
  }
  return (false);
}

void LibcaerWrapper::stopSensor()
{
  stopProcessingThread();
  device_->stop();
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
  LOG_INFO("starting processing thread!");

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
  LOG_INFO("processing thread exited!");
}

void LibcaerWrapper::statsThread()
{
  const auto duration = nanoseconds(static_cast<int>(statsInterval_ * 1000000000));
  while (rclcpp::ok() && keepStatsRunning_.load()) {
    std::unique_lock<std::mutex> lock(statsMutex_);
    statsCv_.wait_for(lock, duration);
    printStatistics();
  }
  LOG_INFO("statistics thread exited!");
}

void LibcaerWrapper::setCoarseFineBias(std::shared_ptr<CoarseFineParameter> p)
{
  const auto & name = p->getName();
  const auto target = p->getBias();
  device_->configSet(p, caerBiasCoarseFineGenerate(p->getBias()));
  // read back from device
  if (p->readBack()) {
    p->setBias(caerBiasCoarseFineParse(device_->configGet(p)));
    const auto & newBias = p->getBias();
    if (newBias.coarseValue != target.coarseValue) {
      LOG_WARN(
        name << " adjusted coarse from target " << static_cast<int32_t>(target.coarseValue)
             << " to " << static_cast<int32_t>(newBias.coarseValue));
    }
    if (newBias.fineValue != target.fineValue) {
      LOG_WARN(
        name << " adjusted fine from target " << static_cast<int32_t>(target.fineValue) << " to "
             << static_cast<int32_t>(newBias.fineValue));
    }
  }
}
void LibcaerWrapper::setVDACBias(std::shared_ptr<VDACParameter> p)
{
  const auto & name = p->getName();
  const auto target = p->getBias();
  device_->configSet(p, caerBiasVDACGenerate(p->getBias()));
  // read back from device
  if (p->readBack()) {
    p->setBias(caerBiasVDACParse(device_->configGet(p)));
    const auto & newBias = p->getBias();
    if (newBias.voltageValue != target.voltageValue) {
      LOG_WARN(
        name << " adjusted voltage from target " << static_cast<int32_t>(target.voltageValue)
             << " to " << static_cast<int32_t>(newBias.voltageValue));
    }
    if (newBias.currentValue != target.currentValue) {
      LOG_WARN(
        name << " adjusted current from target " << static_cast<int32_t>(target.currentValue)
             << " to " << static_cast<int32_t>(newBias.currentValue));
    }
  }
}

void LibcaerWrapper::setShiftedSourceBias(std::shared_ptr<ShiftedSourceParameter> p)
{
  const auto & name = p->getName();
  const auto target = p->getBias();
  device_->configSet(p, caerBiasShiftedSourceGenerate(p->getBias()));
  // read back from device
  if (p->readBack()) {
    p->setBias(caerBiasShiftedSourceParse(device_->configGet(p)));
    const auto & newBias = p->getBias();
    if (
      newBias.refValue != target.refValue || newBias.regValue != target.regValue ||
      newBias.operatingMode != target.operatingMode ||
      newBias.voltageLevel != target.voltageLevel) {
      LOG_WARN(name << " libcaer adjust values of shifted source!");
    }
  }
}

void LibcaerWrapper::setIntegerParameter(std::shared_ptr<IntegerParameter> p)
{
  const auto & name = p->getName();
  const auto targetValue = p->getValue();
  LOG_INFO("setting param: " << name << " to " << p->getValue());
  device_->configSet(p, p->getValue());
  if (p->readBack()) {
    try {
      p->setValue(device_->configGet(p));
      if (p->getValue() != targetValue) {
        LOG_WARN(
          "libcaer adjusted parameter " << p->getName() << " from desired " << targetValue << " to "
                                        << p->getValue());
      }
    } catch (const std::runtime_error & e) {
      LOG_WARN("cannot read back param: " << name << " " << e.what());
    }
  }
  LOG_INFO(
    "set int param: " << int(p->getModAddr()) << ":" << int(p->getParamAddr()) << " to "
                      << targetValue << " set: " << p->getValue());
}

void LibcaerWrapper::setBooleanParameter(std::shared_ptr<BooleanParameter> p)
{
  const auto targetValue = p->getValue();
  device_->configSet(p, static_cast<int32_t>(p->getValue()));
  if (p->readBack()) {
    LOG_INFO("reading back " << p->getName());
    p->setValue(static_cast<bool>(device_->configGet(p)));
    if (p->getValue() != targetValue) {
      LOG_WARN("libcaer could not set parameter " << p->getName());
    }
  }
  LOG_INFO(
    "set bool param: " << int(p->getModAddr()) << ":" << int(p->getParamAddr()) << " to "
                       << (p->getValue() ? "True" : "False"));
}

void LibcaerWrapper::initializeParameters(CallbackHandler * h)
{
  const auto & params = device_->getParameters();
  for (const auto & p : params) {
    const auto rps = p->makeRosParameters(p);
    for (const auto & rp : rps) {
      rp->setParameter(p);
      h->declareParameterCallback(rp);  // create parameter in ros land
    }
    try {
      switch (p->getCaerType()) {
        case CaerParameterType::INTEGER: {
          const auto pc = std::dynamic_pointer_cast<IntegerParameter>(p);
          setIntegerParameter(pc);
          break;
        }
        case CaerParameterType::BOOLEAN: {
          const auto pc = std::dynamic_pointer_cast<BooleanParameter>(p);
          setBooleanParameter(pc);
          break;
        }
        case CaerParameterType::CF_BIAS: {
          const auto cfb = std::dynamic_pointer_cast<CoarseFineParameter>(p);
          setCoarseFineBias(cfb);
          break;
        }
        case CaerParameterType::VDAC_BIAS: {
          const auto vb = std::dynamic_pointer_cast<VDACParameter>(p);
          setVDACBias(vb);
          break;
        }
        case CaerParameterType::SHIFTED_SOURCE_BIAS: {
          const auto ssb = std::dynamic_pointer_cast<ShiftedSourceParameter>(p);
          setShiftedSourceBias(ssb);
          break;
        }
        default:
          BOMB_OUT("invalid parameter type: " << p->getCaerType());
          break;
      }
    } catch (const std::runtime_error & e) {
      LOG_ERROR("error initializing parameter " << p->getName());
    }
  }
}

void LibcaerWrapper::resetTimeStamps() { device_->resetTimeStamps(); }

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
    get_logger(), "in: %9.5f Mev/s, %8.3f MB/s, %5d msgs/s, out: %5d msg/s", recvEventsRate,
    recvByteRate, recvMsgRate, sendMsgRate);
  stats_ = Stats();  // reset statistics
}

}  // namespace libcaer_driver
