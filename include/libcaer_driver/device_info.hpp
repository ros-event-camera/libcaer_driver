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

#ifndef LIBCAER_DRIVER__DEVICE_INFO_HPP_
#define LIBCAER_DRIVER__DEVICE_INFO_HPP_

#include <libcaer/devices/davis.h>

namespace libcaer_driver
{
  struct DeviceInfo
  {
    // --------- common fields indicating device properties
    bool hasDVS{false};
    bool hasIMU{false};
    bool hasAPS{false};
    // -------- common to Davis and DVXplorer
    int16_t deviceID{-1};
    std::string deviceSerialNumber;
    uint8_t deviceUSBBusNumber;
    uint8_t deviceUSBDeviceAddress;
    std::string deviceString;
    int16_t firmwareVersion{-1};
    int16_t logicVersion{-1};
    int16_t chipID{-1};
    bool deviceIsMaster{true};
    bool muxHasStatistics{false};
    int16_t dvsSizeX{0};
    int16_t dvsSizeY{0};
    bool dvsHasStatistics{false};
    enum caer_imu_types imuType;
    bool extInputHasGenerator{false};
    // -------- only valid for Davis
    bool dvsHasPixelFilter{false};
    bool dvsHasBackgroundActivityFilter;
    bool dvsHasROIFilter{false};
    bool dvsHasSkipFilter{false};
    bool dvsHasPolarityFilter{false};
    int16_t apsSizeX{0};
    int16_t apsSizeY{0};
    enum caer_frame_event_color_filter apsColorFilter;
    bool apsHasGlobalShutter{false};
  };

}  // namespace libcaer_driver
#endif  // LIBCAER_DRIVER__DEVICE_INFO_HPP_
