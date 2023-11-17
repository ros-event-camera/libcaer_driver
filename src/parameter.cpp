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

#include <libcaer_driver/parameter.hpp>
#include <libcaercpp/devices/device_discover.hpp>
#include <rclcpp/rclcpp.hpp>

//
// map between the ROS configuration string and libcaer device type.
// add new devices here, but update the device map in libcaer_wrapper.cpp

namespace libcaer_driver
{
static ParameterMap davis{{
  {"PrBp_coarse", Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, false, 2, 0, 7)},
  {"PrBp_fine",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, false, 58, 0, 255)},
  {"PrSFBP_coarse",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, false, 1, 0, 7)},
  {"PrSFBP_fine",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, false, 33, 0, 255)},
  {"DiffBn_coarse",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN, true, 4, 0, 7)},
  {"DiffBn_fine",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN, true, 39, 0, 255)},
  {"ONBn_coarse",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN, true, 6, 0, 255)},
  {"ONBn_fine",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN, true, 200, 0, 255)},
  {"OFFBn_coarse",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN, true, 4, 0, 255)},
  {"OFFBn_fine",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN, true, 0, 0, 255)},
  {"RefrBp_coarse",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP, false, 4, 0, 255)},
  {"RefrBp_fine",
   Parameter(INTEGER, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP, false, 25, 0, 255)},
}};

static ParameterMap dvXplorer{{
  {"dvs_enabled", Parameter(BOOLEAN, DVX_DVS, DVX_DVS_RUN, true, 1, 0, 1)},
  {"bias_sensitivity",
   Parameter(INTEGER, DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, true, 2, 0, 4)},
  // --- polarity
  {"polarity_flatten", Parameter(BOOLEAN, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_FLATTEN, true, 0, 0, 1)},
  {"polarity_on_only", Parameter(BOOLEAN, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_ON_ONLY, true, 0, 0, 1)},
  {"polarity_off_only",
   Parameter(BOOLEAN, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_OFF_ONLY, true, 0, 0, 1)},
  // --- subsampling
  {"subsample_enable",
   Parameter(BOOLEAN, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_ENABLE, false, 0, 0, 1)},
  {"subsample_vertical",
   Parameter(INTEGER, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_VERTICAL, true, 0, 0, 7)},
  {"subsample_horizontal",
   Parameter(INTEGER, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL, true, 0, 0, 7)},
  // --- ROI
  {"roi_enabled",
   Parameter(BOOLEAN, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_EVENT_OFF_ONLY, true, 0, 0, 1)},
  {"roi_start_column",
   Parameter(INTEGER, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_X_START_ADDRESS, true, 0, 0, 639)},
  {"roi_start_row",
   Parameter(INTEGER, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS, true, 0, 0, 479)},
  {"roi_end_column",
   Parameter(INTEGER, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_X_END_ADDRESS, true, 639, 0, 639)},
  {"roi_end_row",
   Parameter(INTEGER, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS, true, 479, 0, 479)},
  // --- IMU
  {"accelerometer_enabled", Parameter(BOOLEAN, DVX_IMU, DVX_IMU_RUN_ACCELEROMETER, true, 1, 0, 1)},
  {"gyro_enabled", Parameter(BOOLEAN, DVX_IMU, DVX_IMU_RUN_GYROSCOPE, true, 1, 0, 1)},
  {"imu_acc_scale", Parameter(INTEGER, DVX_IMU, DVX_IMU_ACCEL_RANGE, true, 1, 0, 3)},
  {"imu_gyro_scale", Parameter(INTEGER, DVX_IMU, DVX_IMU_GYRO_RANGE, true, 2, 0, 4)},
}};

static const std::map<int, const ParameterMap &> maps = {
  {CAER_DEVICE_DVXPLORER, dvXplorer}, {CAER_DEVICE_DAVIS, davis}};

const ParameterMap & Parameter::getMap(int deviceType)
{
  const auto & it = maps.find(deviceType);
  if (it == maps.end()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("driver"), "cannot find parameter map for device type " << deviceType);
    throw(std::runtime_error("no map for device type"));
  }
  return (it->second);
}

}  // namespace libcaer_driver
