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

#ifndef LIBCAER_DRIVER__DEVICE__DEVICE_HPP_
#define LIBCAER_DRIVER__DEVICE__DEVICE_HPP_

#include <libcaer_driver/callback_handler.hpp>
#include <libcaer_driver/device_info.hpp>
#include <libcaer_driver/parameter/parameter.hpp>
#include <libcaercpp/devices/device.hpp>
#include <memory>
#include <string>
#include <vector>

namespace libcaer_driver
{
using Devices = std::vector<std::pair<std::string, std::string>>;

class Device
{
public:
  virtual ~Device() = default;
  // --------- to be implemented by derived classe
  virtual void resetTimeStamps() = 0;
  virtual void setExposureTime(int32_t t) = 0;
  virtual int32_t getExposureTime() const = 0;

  // -------- own methods
  const Parameters & getParameters() const { return (*parameters_); }
  const DeviceInfo & getDeviceInfo() const { return (deviceInfo_); }

  void setDeviceInfo(DeviceInfo & di) { deviceInfo_ = di; }
  void setDevice(std::unique_ptr<libcaer::devices::device> & d) { device_ = std::move(d); }

  void start(CallbackHandler * h);
  void stop();

  void configSet(const std::shared_ptr<Parameter> & p, uint32_t value);
  uint32_t configGet(const std::shared_ptr<Parameter> & p);

  std::unique_ptr<libcaer::events::EventPacketContainer> dataGet() { return (device_->dataGet()); }

  // ----------- static methods
  static std::shared_ptr<Device> newInstance(
    const std::string & devType, int16_t devId, const std::string & restrictSerialNumber);
  static Devices logAllDevices();

protected:
  DeviceInfo deviceInfo_;
  std::shared_ptr<Parameters> parameters_;
  std::unique_ptr<libcaer::devices::device> device_;
  bool deviceRunning_{false};
};

}  // namespace libcaer_driver
#endif  // LIBCAER_DRIVER__DEVICE__DEVICE_HPP_
