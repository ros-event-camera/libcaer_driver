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

#include <libcaer/devices/dvxplorer.h>

#include <libcaer_driver/device/dvxplorer.hpp>
#include <libcaer_driver/logging.hpp>
#include <libcaer_driver/parameter/boolean_parameter.hpp>
#include <libcaer_driver/parameter/integer_parameter.hpp>

namespace libcaer_driver
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("device")); }
//
// The following definitions are used for a more compact notation when building the
// parameter list.
//
static auto Bool = [](const std::string & n, int8_t ma, uint8_t pa, bool def) {
  return (std::make_shared<BooleanParameter>(n, ma, pa, def));
};

static auto IntBias = [](const std::string & n, uint8_t pa, int32_t def, int32_t vn, int32_t vx) {
  // for some reason the bias sensitivity cannot be read from the device with libcaer
  return (std::make_shared<IntegerParameter>(n, DVX_DVS_CHIP_BIAS, pa, def, vn, vx, false));
};

static auto IntChip = [](const std::string & n, uint8_t pa, int32_t def, int32_t vn, int32_t vx) {
  return (std::make_shared<IntegerParameter>(n, DVX_DVS_CHIP, pa, def, vn, vx, true));
};

static auto IntCrop = [](const std::string & n, uint8_t pa, int32_t def, int32_t vn, int32_t vx) {
  return (std::make_shared<IntegerParameter>(n, DVX_DVS_CHIP_CROPPER, pa, def, vn, vx, true));
};

static auto IntImu = [](const std::string & n, uint8_t pa, int32_t def, int32_t vn, int32_t vx) {
  return (std::make_shared<IntegerParameter>(n, DVX_IMU, pa, def, vn, vx, true));
};

static std::shared_ptr<Parameters> make_dvxplorer_parameters()
{
  auto sp = std::make_shared<Parameters>();
  auto * p = sp.get();
  p->add(Bool("dvs_enabled", DVX_DVS, DVX_DVS_RUN, true));
  p->add(IntBias("bias_sensitivity", DVX_DVS_CHIP_BIAS_SIMPLE, 2, 0, 4));
  p->add(Bool("polarity_flatten", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_FLATTEN, false));
  p->add(Bool("polarity_on_only", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_ON_ONLY, false));
  p->add(Bool("polarity_off_only", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_OFF_ONLY, false));
  p->add(Bool("subsample_enabled", DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_ENABLE, false));
  p->add(IntChip("subsample_vertical", DVX_DVS_CHIP_SUBSAMPLE_VERTICAL, 0, 0, 7));
  p->add(IntChip("subsample_horizontal", DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL, 0, 0, 7));
  p->add(Bool("roi_enabled", DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_ENABLE, false));
  p->add(IntCrop("roi_start_col", DVX_DVS_CHIP_CROPPER_X_START_ADDRESS, 0, 0, 639));
  p->add(IntCrop("roi_start_row", DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS, 0, 0, 479));
  p->add(IntCrop("roi_end_col", DVX_DVS_CHIP_CROPPER_X_END_ADDRESS, 639, 0, 639));
  p->add(IntCrop("roi_end_row", DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS, 479, 0, 479));
  p->add(Bool("imu_accel_enabled", DVX_IMU, DVX_IMU_RUN_ACCELEROMETER, true));
  p->add(Bool("imu_gyro_enabled", DVX_IMU, DVX_IMU_RUN_GYROSCOPE, true));
  p->add(IntImu("imu_accel_scale", DVX_IMU_ACCEL_RANGE, 1, 0, 3));
  p->add(IntImu("imu_gyro_scale", DVX_IMU_GYRO_RANGE, 2, 0, 4));
  return (sp);
}

DvXplorer::DvXplorer() { parameters_ = make_dvxplorer_parameters(); }

void DvXplorer::resetTimeStamps()
{
  try {
    device_->configSet(DVX_MUX, DVX_MUX_TIMESTAMP_RESET, 1);
  } catch (const std::runtime_error &) {
    // resetting timestamp throws error for DvXplorer Mini
    LOG_WARN("Could not reset time stamps.");
  }
}
}  // namespace libcaer_driver
