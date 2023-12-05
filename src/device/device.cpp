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

#include <libcaer_driver/device/davis.hpp>
#include <libcaer_driver/device/device.hpp>
#include <libcaer_driver/device/dvxplorer.hpp>
#include <libcaer_driver/logging.hpp>
#include <libcaer_driver/parameter/boolean_parameter.hpp>
#include <libcaer_driver/parameter/coarse_fine_parameter.hpp>
#include <libcaer_driver/parameter/integer_parameter.hpp>
#include <libcaer_driver/parameter/shifted_source_parameter.hpp>
#include <libcaer_driver/parameter/vdac_parameter.hpp>
#include <libcaercpp/devices/davis.hpp>
#include <libcaercpp/devices/device_discover.hpp>
#include <libcaercpp/devices/dvxplorer.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>  // for logging only

namespace libcaer_driver
{
// need to use free function for usb disconnect handling
static void device_disconnected(void * ptr)
{
  reinterpret_cast<CallbackHandler *>(ptr)->deviceDisconnectedCallback();
}

// local logger handle
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("device")); }

template <class T>
static void copy_common_fields(DeviceInfo * out, const T & info)
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
static void copy_specific_fields(DeviceInfo *, const T &)
{
}

template <typename T>
std::unique_ptr<T> open_dev(int16_t deviceId, const std::string & restrictSN, DeviceInfo * di)
{
  auto p = std::make_unique<T>(deviceId, 0 /*usb bus*/, 0 /*usb dev*/, restrictSN);
  const auto info = reinterpret_cast<T *>(p.get())->infoGet();
  copy_common_fields(di, info);
  // deduce device info from return type of infoGet
  using I = typename std::result_of<decltype (&T::infoGet)(T)>::type;
  copy_specific_fields<I>(di, info);
  return (p);
}

static const std::map<std::string, int> string_to_device_type = {
  {"dvxplorer", CAER_DEVICE_DVXPLORER}, {"davis", CAER_DEVICE_DAVIS}};

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

Devices Device::logAllDevices()
{
  Devices devices;
  const auto all_devices = libcaer::devices::discover::all();
  if (all_devices.empty()) {
    LOG_ERROR("found no devices!");
  } else {
    LOG_INFO("found " << all_devices.size() << " device(s)");
    for (const auto & dev : all_devices) {
      std::string devInfo("unknown device type");
      switch (dev.deviceType) {
        case CAER_DEVICE_DAVIS:
          devices.push_back({"davis", dev.deviceInfo.davisInfo.deviceSerialNumber});
          devInfo = "DAVIS SN: " + std::string(dev.deviceInfo.davisInfo.deviceSerialNumber);
          break;
        case CAER_DEVICE_DVXPLORER:
          devInfo = "DVXPLORER SN: " + std::string(dev.deviceInfo.dvXplorerInfo.deviceSerialNumber);
          devices.push_back({"dvxplorer", dev.deviceInfo.dvXplorerInfo.deviceSerialNumber});
          break;
        default:
          break;
      }
      LOG_INFO_FMT("found device: %-30s of type %d", devInfo.c_str(), dev.deviceType);
    }
  }
  return (devices);
}

void Device::start(CallbackHandler * h)
{
  if (!deviceRunning_) {
    device_->dataStart(nullptr, nullptr, nullptr, device_disconnected, h);
    deviceRunning_ = true;
  }
}

void Device::stop()
{
  if (deviceRunning_) {
    device_->dataStop();
    deviceRunning_ = false;
    LOG_INFO("stopped sensor");
    deviceRunning_ = false;
  }
}

void Device::configSet(const std::shared_ptr<Parameter> & p, uint32_t value)
{
  device_->configSet(p->getModAddr(), p->getParamAddr(), value);
}

uint32_t Device::configGet(const std::shared_ptr<Parameter> & p)
{
  return (device_->configGet(p->getModAddr(), p->getParamAddr()));
}

std::shared_ptr<Device> Device::newInstance(
  const std::string & deviceType, int16_t deviceId, const std::string & restrictSN)
{
  const auto dt_it = string_to_device_type.find(deviceType);
  if (dt_it == string_to_device_type.end()) {
    BOMB_OUT("unknown device type: " << deviceType);
  }
  DeviceInfo di;
  std::shared_ptr<Device> dev;
  std::unique_ptr<libcaer::devices::device> libCaerDev;
  const auto devType = dt_it->second;
  switch (devType) {
    case CAER_DEVICE_DAVIS:
      libCaerDev = open_dev<libcaer::devices::davis>(deviceId, restrictSN, &di);
      di.hasAPS = true;
      dev = std::make_shared<Davis>(di.chipID);
      break;
    case CAER_DEVICE_DVXPLORER:
      libCaerDev = open_dev<libcaer::devices::dvXplorer>(deviceId, restrictSN, &di);
      dev = std::make_shared<DvXplorer>();
      break;
    default:
      BOMB_OUT("device type not fully implemented: " << deviceType);
      break;
  }
  if (!libCaerDev) {
    return (nullptr);  // libcaer could not open the device
  }
  libCaerDev->sendDefaultConfig();
  libCaerDev->configSet(
    CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
  dev->setDeviceInfo(di);
  dev->setDevice(libCaerDev);
  return (dev);
}

}  // namespace libcaer_driver
