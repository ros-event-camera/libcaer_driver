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
// ------------- local static functions and maps ------------
//
// map between the ROS configuration string and libcaer device type.
// add new devices here, but also update the other maps!

static const std::map<std::string, int> devices = {
  {"dvxplorer", CAER_DEVICE_DVXPLORER}, {"davis", CAER_DEVICE_DAVIS}};
//

struct DeviceConfig
{
  DeviceConfig(int8_t biasModAddr, int8_t apsModAddr, int8_t dvsModAddr)
  : bias(biasModAddr), aps(apsModAddr), dvs(dvsModAddr)
  {
  }
  int8_t bias{0};
  int8_t aps{0};
  int8_t dvs{0};
};

static const std::map<int, const DeviceConfig> DEVICE_CONFIGURATIONS = {
  {CAER_DEVICE_DAVIS, {DAVIS_CONFIG_BIAS, DAVIS_CONFIG_APS, DAVIS_CONFIG_DVS}}};

constexpr ParamInfo::ParamType BIAS = ParamInfo::ParamType::BIAS;
static const std::map<int16_t, std::map<std::string, const ParamInfo>> ALL_PARAMETERS = {
  {CAER_DEVICE_DAVIS,
   {
     {"PrBp_coarse", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_PRBP, false, 2, 0, 7)},
     {"PrBp_fine", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_PRBP, false, 58, 0, 255)},
     {"PrSFBP_coarse", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, false, 1, 0, 7)},
     {"PrSFBP_fine", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, false, 33, 0, 255)},
     {"DiffBn_coarse", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_DIFFBN, true, 4, 0, 7)},
     {"DiffBn_fine", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_DIFFBN, true, 39, 0, 255)},
     {"ONBn_coarse", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_ONBN, true, 6, 0, 255)},
     {"ONBn_fine", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_ONBN, true, 200, 0, 255)},
     {"OFFBn_coarse", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_OFFBN, true, 4, 0, 255)},
     {"OFFBn_fine", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_OFFBN, true, 0, 0, 255)},
     {"RefrBp_coarse", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_REFRBP, false, 4, 0, 255)},
     {"RefrBp_fine", ParamInfo(BIAS, DAVIS240_CONFIG_BIAS_REFRBP, false, 25, 0, 255)},
   }}};  // needs 3 closing braces here

// local logger handle
static rclcpp::Logger logger() { return (rclcpp::get_logger("driver")); }

// need to use free function for usb disconnect handling
static void device_disconnected(void * ptr)
{
  reinterpret_cast<LibcaerWrapper *>(ptr)->deviceDisconnected();
}

//
// -------------- free functions  -----------------------
//

size_t LibcaerWrapper::convert_to_mono(
  std::vector<uint8_t> * data, uint64_t timeBase,
  const libcaer::events::PolarityEventPacket & packet)
{
  const size_t BYTES_PER_ENCODED_EVENT = 8;
  const size_t n = packet.getEventNumber() * BYTES_PER_ENCODED_EVENT;
  const size_t oldSize = data->size();
  resize_hack(*data, oldSize + n);

  uint64_t * p = reinterpret_cast<uint64_t *>(data->data());
  for (int32_t i = 0; i < packet.getEventNumber(); i++, p++) {
    const auto & e = packet[i];
    const auto t = e.getTimestamp64(packet);
    const uint64_t dt = t - timeBase;
    *p = static_cast<uint64_t>(e.getPolarity()) << 63 | static_cast<uint64_t>(e.getY()) << 48 |
         static_cast<uint64_t>(e.getX()) << 32 | dt;
  }
  return (packet.getEventNumber());
}

void LibcaerWrapper::frame_to_ros_msg(
  const libcaer::events::FrameEventPacket & packet, std::string * encoding,
  std::vector<uint8_t> * out, uint32_t * width, uint32_t * height, uint32_t * step)
{
  const auto & frame = packet[0];  // grab first frame in packet
  const auto nc = frame.getChannelNumber();
  *height = frame.getLengthY();
  *width = frame.getLengthX();

  const uint16_t * data = frame.getPixelArrayUnsafe();
  uint32_t numChan = static_cast<uint32_t>(nc);
  switch (frame.getChannelNumber()) {
    case libcaer::events::FrameEvent::colorChannels::GRAYSCALE:
      *encoding = "mono8";
      break;
    case libcaer::events::FrameEvent::colorChannels::RGB:
      *encoding = "rgb8";
      break;
    case libcaer::events::FrameEvent::colorChannels::RGBA:
      *encoding = "rgba8";
      break;
    default:
      RCLCPP_ERROR_STREAM(
        logger(), "invalid number of channels for frame: " << static_cast<uint32_t>(nc));
      throw(std::runtime_error("invalid number of channels for frame"));
  }

  const uint32_t stride = numChan * (*width);
  *step = stride;
  out->resize(stride * (*height));
  for (uint32_t y = 0; y < (*height) * numChan; y++) {
    for (uint32_t x = 0; x < *width; x++) {
      (*out)[y * (*width) + x] = data[y * (*width) + x] >> 8;  // convert from 16bit to 8bit.
    }
  }
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
  if (statsThread_) {
    {
      keepStatsRunning_.store(false);
      std::unique_lock<std::mutex> lock(statsMutex_);
      statsCv_.notify_all();
    }
    statsThread_->join();
    statsThread_.reset();
  }
  device_.reset();
}

void LibcaerWrapper::deviceDisconnected()
{
  RCLCPP_ERROR(logger(), "device disconnected!");
  stopSensor();
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

void LibcaerWrapper::initialize(
  const std::string & devType, int deviceId, const std::string & restrictSN)
{
  const auto dev_it = devices.find(devType);
  if (dev_it == devices.end()) {
    throw(std::runtime_error("unsupported device configured: " + devType));
  }
  deviceType_ = dev_it->second;
  const auto all_devices = libcaer::devices::discover::all();
  RCLCPP_INFO_STREAM(logger(), "found " << all_devices.size() << " device(s)");

  const int num_tries = 5;
  for (int i = 0; i < num_tries; i++) {
    auto devices = libcaer::devices::discover::device(deviceType_);
    RCLCPP_INFO_STREAM(
      logger(),
      "found " << devices.size() << " device(s) of type " << devType << "(" << deviceType_ << ")");
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

int LibcaerWrapper::setBias(const std::string & name, const ParamInfo & p, int value)
{
  const auto devconfig_it = DEVICE_CONFIGURATIONS.find(deviceType_);
  if (devconfig_it == DEVICE_CONFIGURATIONS.end()) {
    RCLCPP_ERROR_STREAM(logger(), "no config defined for device " << deviceType_);
    throw(std::runtime_error("no config defined for device!"));
  }
  // first read the current bias to get either coarse or fine value, whichever
  // is not being set right now
  const auto modAddr = devconfig_it->second.bias;
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

const std::map<std::string, const ParamInfo> & LibcaerWrapper::getParameters()
{
  const auto map_it = ALL_PARAMETERS.find(deviceType_);
  if (map_it == ALL_PARAMETERS.end()) {
    RCLCPP_ERROR_STREAM(logger(), "no parameters defined for device " << deviceType_);
    throw(std::runtime_error("no parameters defined for device!"));
  }
  return (map_it->second);
}

int LibcaerWrapper::setParameter(const std::string & name, int value)
{
  const auto & parameters = getParameters();
  const auto param_it = parameters.find(name);
  if (param_it == parameters.end()) {
    RCLCPP_ERROR_STREAM(logger(), "param " << name << "not defined for device " << deviceType_);
    throw(std::runtime_error("param not defined for device!"));
  }
  const auto & p = param_it->second;
  int actualValue{0};
  switch (p.type) {
    case ParamInfo::BIAS:
      actualValue = setBias(name, p, value);
      break;
    default:
      RCLCPP_ERROR_STREAM(logger(), "param " << name << "has invalid param type");
      throw(std::runtime_error("param not defined for device!"));
  }
  return (actualValue);
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
