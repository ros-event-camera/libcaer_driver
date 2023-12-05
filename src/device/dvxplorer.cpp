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
//
// The following definitions are used for a more compact notation when building the
// parameter list. Add more as needed.
//
static void addBool(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, bool def,
  bool rb = false)
{
  p->push_back(std::make_shared<BooleanParameter>(name, modAddr, paramAddr, def, rb));
}

static void addInt(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, int32_t def,
  int32_t vn, int32_t vx, bool rb = true)
{
  p->push_back(std::make_shared<IntegerParameter>(name, modAddr, paramAddr, def, vn, vx, rb));
}

// ---------- specializations for device
static void addIntDvXChip(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  return (addInt(p, name, DVX_DVS_CHIP, paramAddr, def, vn, vx));
}

static void addIntDvXCrop(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  // reading back the row values returns the original ones even though the parameters are set...
  return (addInt(p, name, DVX_DVS_CHIP_CROPPER, paramAddr, def, vn, vx, false));
}
static void addIntDvXImu(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  return (addInt(p, name, DVX_IMU, paramAddr, def, vn, vx));
}

static std::shared_ptr<Parameters> make_dvxplorer_parameters()
{
  auto sp = std::make_shared<Parameters>();
  auto * p = sp.get();
  addBool(p, "dvs_enabled", DVX_DVS, DVX_DVS_RUN, true);
  // for some reason the bias sensitivity cannot be read from the device with libcaer
  addInt(p, "bias_sensitivity", DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, 2, 0, 4, false);
  addBool(p, "polarity_flatten", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_FLATTEN, false);
  addBool(p, "polarity_on_only", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_ON_ONLY, false);
  addBool(p, "polarity_off_only", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_OFF_ONLY, false);
  addBool(p, "subsample_enabled", DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_ENABLE, false);
  addIntDvXChip(p, "subsample_vertical", DVX_DVS_CHIP_SUBSAMPLE_VERTICAL, 0, 0, 7);
  addIntDvXChip(p, "subsample_horizontal", DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL, 0, 0, 7);
  addBool(p, "roi_enabled", DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_ENABLE, false);
  addIntDvXCrop(p, "roi_start_col", DVX_DVS_CHIP_CROPPER_X_START_ADDRESS, 0, 0, 639);
  addIntDvXCrop(p, "roi_start_row", DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS, 0, 0, 479);
  addIntDvXCrop(p, "roi_end_col", DVX_DVS_CHIP_CROPPER_X_END_ADDRESS, 639, 0, 639);
  addIntDvXCrop(p, "roi_end_row", DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS, 479, 0, 479);
  addBool(p, "imu_accel_enabled", DVX_IMU, DVX_IMU_RUN_ACCELEROMETER, true);
  addBool(p, "imu_gyro_enabled", DVX_IMU, DVX_IMU_RUN_GYROSCOPE, true);
  addIntDvXImu(p, "imu_accel_scale", DVX_IMU_ACCEL_RANGE, 1, 0, 3);
  addIntDvXImu(p, "imu_gyro_scale", DVX_IMU_GYRO_RANGE, 2, 0, 4);
  return (sp);
};

DvXplorer::DvXplorer() { parameters_ = make_dvxplorer_parameters(); }

void DvXplorer::resetTimeStamps() { device_->configSet(DVX_MUX, DVX_MUX_TIMESTAMP_RESET, 1); }

}  // namespace libcaer_driver
