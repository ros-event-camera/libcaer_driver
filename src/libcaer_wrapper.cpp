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
void copy_common_fields(LibcaerWrapper::DevInfo * out, const T & info)
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
void copy_specific_fields(LibcaerWrapper::DevInfo *, const T &)
{
  // nothing specific to copy (dvXplorer)
}

// specialize template for davis
template <>
void copy_specific_fields(LibcaerWrapper::DevInfo * out, const caer_davis_info & info)
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

template <class T, class I>
std::unique_ptr<T> open_dev(
  rclcpp::Logger logger, int16_t deviceId, const std::string & restrictSN,
  LibcaerWrapper::DevInfo * di)
{
  auto p = std::make_unique<T>(deviceId, 0 /*usb bus*/, 0 /*usb dev*/, restrictSN);
  const auto info = reinterpret_cast<T *>(p.get())->infoGet();
  copy_common_fields(di, info);
  copy_specific_fields<I>(di, info);
  RCLCPP_INFO(
    logger, "opened %s: id: %d, master: %d, res(%d, %d), logic version: %d", info.deviceString,
    info.deviceID, info.deviceIsMaster, info.dvsSizeX, info.dvsSizeY, info.logicVersion);
  return (p);
}

static std::unique_ptr<libcaer::devices::device> open_device(
  rclcpp::Logger logger, int16_t deviceId, const caer_device_discovery_result & dr,
  const std::string & restrictSN, LibcaerWrapper::DevInfo * info)
{
  std::unique_ptr<libcaer::devices::device> p;

  switch (dr.deviceType) {
    case CAER_DEVICE_DAVIS:
      p = open_dev<libcaer::devices::davis, caer_davis_info>(logger, deviceId, restrictSN, info);
      break;
    case CAER_DEVICE_DVXPLORER:
      p = open_dev<libcaer::devices::dvXplorer, caer_dvx_info>(logger, deviceId, restrictSN, info);
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
      device_ = open_device(logger(), deviceId, dev, restrictSN, &devInfo_);
      if (device_) {
        break;
      }
    }
    if (!device_) {
      RCLCPP_ERROR_STREAM(
        logger(), "cannot open device of type " << devType << " on attempt " << i + 1
                                                << ", retrying " << num_tries - i - 1
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
  RCLCPP_INFO_STREAM(logger(), "device opened successfully");
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
      std::cout << "got polarity event!" << std::endl;
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

Value LibcaerWrapper::setCourseFineBias(
  const std::string & name, std::shared_ptr<CoarseFineParameter> p, int32_t targetBias)
{
  p->setBias(name, targetBias);      // updates the bias struct internal to the parameter
  const auto & bias = p->getBias();  // get the updated bias struct
  // send updated bias to device
  device_->configSet(p->getModAddr(), p->getParamAddr(), caerBiasCoarseFineGenerate(bias));
  // read back from device
  const auto newBias = caerBiasCoarseFineParse(configGet(p->getModAddr(), p->getParamAddr()));
  // pick the correct bias
  const auto newValue = p->isCoarseBias(name) ? newBias.coarseValue : newBias.fineValue;
  // update the bias struct with what is actually on the device
  p->setBias(name, newValue);
  if (newValue != static_cast<uint8_t>(targetBias)) {
    RCLCPP_WARN_STREAM(
      logger(),
      name << " adjusted from target " << targetBias << " to " << static_cast<int32_t>(newValue));
  }
  return (Value(static_cast<int32_t>(newValue)));
}

Value LibcaerWrapper::setVDACBias(
  const std::string & name, std::shared_ptr<VDACParameter> p, int32_t targetBias)
{
  // see setCourseFineBias() for comments
  p->setBias(name, targetBias);
  const auto & bias = p->getBias();
  device_->configSet(p->getModAddr(), p->getParamAddr(), caerBiasVDACGenerate(bias));
  const auto newBias = caerBiasVDACParse(configGet(p->getModAddr(), p->getParamAddr()));
  const auto newValue = p->isCurrentBias(name) ? newBias.currentValue : newBias.voltageValue;
  p->setBias(name, newValue);
  if (newValue != static_cast<uint8_t>(targetBias)) {
    RCLCPP_WARN_STREAM(
      logger(),
      name << " adjusted from target " << targetBias << " to " << static_cast<int32_t>(newValue));
  }
  return (Value(static_cast<int32_t>(newValue)));
}

Value LibcaerWrapper::setIntegerParameter(
  const std::string & name, std::shared_ptr<IntegerParameter> p, int32_t targetValue)
{
  RCLCPP_INFO_STREAM(logger(), "setting param: " << name << " to " << p->getValue(name).get<int32_t>());
  p->setValue(targetValue);
  device_->configSet(p->getModAddr(), p->getParamAddr(), p->getValue(name).get<int32_t>());
  uint32_t actualValue = targetValue;
  try {
    actualValue = configGet(p->getModAddr(), p->getParamAddr());
    p->setValue(actualValue);
  } catch (const std::runtime_error & e) {
    RCLCPP_INFO_STREAM(logger(), "cannot read back param: " << name << " " << e.what());
  }
  return (p->getValue(name));
}

bool LibcaerWrapper::setBooleanParameter(
  std::shared_ptr<BooleanParameter> p, bool targetValue)
{
  p->setValue(targetValue);
  device_->configSet(p->getModAddr(), p->getParamAddr(), p->getValue());
  return (targetValue); // assume the setting worked....
}

uint32_t LibcaerWrapper::configGet(int8_t modAddr, uint8_t paramAddr)
{
  uint32_t v;
  device_->configGet(modAddr, paramAddr, &v);
  return (v);
}

void LibcaerWrapper::initializeParameters(CallbackHandler * h)
{
  const auto & params = getParameters();
  for (const auto & p : params) {
    const auto rps = p->getRosParameters();
    for (const auto & rp : rps) {
      h->declareParameter(p, rp);
    }
    try {
      switch (p->getCaerType()) {
        case CaerParameterType::INTEGER: {
          const auto pc = std::dynamic_pointer_cast<IntegerParameter>(p);
          setIntegerParameter(pc->getName(), pc, pc->getValue(pc->getName()).get<int32_t>());
          break;
        }
        case CaerParameterType::BOOLEAN: {
          const auto pc = std::dynamic_pointer_cast<BooleanParameter>(p);
          setBooleanParameter(pc, pc->getValue());
          break;
        }
        case CaerParameterType::CF_BIAS: {
          //auto cfb = std::dynamic_pointer_cast<CoarseFineParameter>(p);
          break;
        }
        case CaerParameterType::VDAC_BIAS: {
          break;
        }
        default:
          throw(std::runtime_error("invalid parameter type!"));
          break;
      }
    } catch (const std::runtime_error &e) {
      RCLCPP_WARN_STREAM(logger(), "error initializing parameter " << p->getName());
    }
  }
}

const Parameters & LibcaerWrapper::getParameters()
{
  try {
    auto ptr = Parameter::instanceOfParameters(deviceType_, devInfo_.chipID);
    return (*ptr);
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
