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

#include <libcaer_driver/boolean_parameter.hpp>
#include <libcaer_driver/coarse_fine_parameter.hpp>
#include <libcaer_driver/integer_parameter.hpp>
#include <libcaer_driver/shifted_source_parameter.hpp>
#include <libcaer_driver/parameter.hpp>
#include <libcaer_driver/vdac_parameter.hpp>
#include <libcaercpp/devices/device_discover.hpp>
#include <rclcpp/rclcpp.hpp>

//
// Map between the ROS parameters and libcaer parameters for configSet().
// Add new devices here, but update the device map in libcaer_wrapper.cpp

namespace libcaer_driver
{
static rclcpp::Logger logger() { return (rclcpp::get_logger("driver")); }

static std::shared_ptr<Parameters> parameters;

static void addBool(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, bool def)
{
  p->push_back(std::make_shared<BooleanParameter>(name, modAddr, paramAddr, def));
}

static void addInt(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, int32_t def,
  int32_t vn, int32_t vx)
{
  p->push_back(std::make_shared<IntegerParameter>(name, modAddr, paramAddr, def, vn, vx));
}

static void addCFB(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, uint8_t defC,
  uint8_t defF, bool sexN)
{
  p->push_back(std::make_shared<CoarseFineParameter>(name, modAddr, paramAddr, defC, defF, sexN));
}

static void addCFBn(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, uint8_t defC,
  uint8_t defF)
{
  addCFB(p, name, modAddr, paramAddr, defC, defF, true);
}

static void addCFBp(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, uint8_t defC,
  uint8_t defF)
{
  addCFB(p, name, modAddr, paramAddr, defC, defF, false);
}

static void addV(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, uint8_t defV,
  uint8_t defC)
{
  p->push_back(std::make_shared<VDACParameter>(name, modAddr, paramAddr, defV, defC));
}

static void addSS(
  Parameters * p, const std::string & name, int8_t modAddr, uint8_t paramAddr, uint8_t ref,
  uint8_t reg, uint8_t om) {
  p->push_back(std::make_shared<ShiftedSourceParameter>(name, modAddr, paramAddr, ref, reg, om));
}

// ---------- specializations for various devices
static void addIntDavisImu(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  return (addInt(p, name, DAVIS_CONFIG_IMU, paramAddr, def, vn, vx));
}
static void addIntDavisAPS(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  return (addInt(p, name, DAVIS_CONFIG_APS, paramAddr, def, vn, vx));
}

static void addIntDVS(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  return (addInt(p, name, DAVIS_CONFIG_DVS, paramAddr, def, vn, vx));
}

static void addBoolDVS(
  Parameters * p, const std::string & name, uint8_t paramAddr, bool v)
{
  addBool(p, name, DAVIS_CONFIG_DVS, paramAddr, v);
}

static void addCFBpDavis(
  Parameters * p, const std::string & name, uint8_t paramAddr, uint8_t defC, uint8_t defF)
{
  addCFBp(p, name, DAVIS_CONFIG_BIAS, paramAddr, defC, defF);
}

static void addCFBnDavis(
  Parameters * p, const std::string & name, uint8_t paramAddr, uint8_t defC, uint8_t defF)
{
  addCFBn(p, name, DAVIS_CONFIG_BIAS, paramAddr, defC, defF);
}

static void addVDavis(
  Parameters * p, const std::string & name, uint8_t paramAddr, uint8_t defV, uint8_t defC)
{
  addV(p, name, DAVIS_CONFIG_BIAS, paramAddr, defV, defC);
}

static void addIntDvXChip(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  return (addInt(p, name, DVX_DVS_CHIP, paramAddr, def, vn, vx));
}

static void addIntDvXCrop(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  return (addInt(p, name, DVX_DVS_CHIP_CROPPER, paramAddr, def, vn, vx));
}
static void addIntDvXImu(
  Parameters * p, const std::string & name, uint8_t paramAddr, int32_t def, int32_t vn, int32_t vx)
{
  return (addInt(p, name, DVX_IMU, paramAddr, def, vn, vx));
}

void make_davis_common_parameters(Parameters * p)
{
  // ----------- IMU
  addBool(p, "imu_acc_enabled", DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, true);
  addBool(p, "imu_gyro_enabled", DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, true);
  addBool(p, "imu_temp_enabled", DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, true);
  addIntDavisImu(p, "imu_acc_scale", DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, 3, 0, 3);
  addIntDavisImu(p, "imu_gyro_scale", DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, 3, 0, 3);
  addIntDavisImu(p, "imu_low_pass_filter", DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER, 1, 0, 6);
  // note: set rate divider to 7 when lowpass filter is enabled (!=0), otherwise set to 0
  addIntDavisImu(p, "imu_sample_rate_divider", DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 7, 0, 7);
  // ----------- APS
  addBool(p, "aps_enabled", DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, true);
  addIntDavisAPS(p, "aps_exposure", DAVIS_CONFIG_APS_EXPOSURE, 5000, 0, 1000000);
  addIntDavisAPS(p, "aps_frame_mode", DAVIS_CONFIG_APS_FRAME_MODE, 0, 0, 2);
  addIntDavisAPS(p, "aps_frame_interval", DAVIS_CONFIG_APS_FRAME_INTERVAL, 25000, 0, 8388607);
  // ---------- DVS
  addBool(p, "dvs_enabled", DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, true);
};

void make_davis240c_parameters(Parameters * p)
{
  addCFBpDavis(p, "PrBp", DAVIS240_CONFIG_BIAS_PRBP, 2, 58);
  addCFBpDavis(p, "PrSFBp", DAVIS240_CONFIG_BIAS_PRSFBP, 1, 33);
  addCFBnDavis(p, "DiffBn", DAVIS240_CONFIG_BIAS_DIFFBN, 4, 39);
  addCFBnDavis(p, "ONBn", DAVIS240_CONFIG_BIAS_ONBN, 6, 200);
  addCFBnDavis(p, "OFFBn", DAVIS240_CONFIG_BIAS_OFFBN, 4, 0);
  addCFBpDavis(p, "RefrBp", DAVIS240_CONFIG_BIAS_REFRBP, 4, 25);
}

void make_davis346b_parameters(Parameters * p)
{
  addVDavis(p, "", DAVIS346_CONFIG_BIAS_APSOVERFLOWLEVEL, 27, 6);
  addVDavis(p, "", DAVIS346_CONFIG_BIAS_APSCAS, 21, 6);
  addVDavis(p, "ADC_RefHigh", DAVIS346_CONFIG_BIAS_ADCREFHIGH, 27, 7);
  addVDavis(p, "ADC_RefLow", DAVIS346_CONFIG_BIAS_ADCREFLOW, 1, 7);
  addVDavis(p, "", DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE, 21, 7);
  addVDavis(p, "", DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE, 21, 7);

  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_LOCALBUFBN, 5, 164);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_PADFOLLBN, 7, 215);
  addCFBnDavis(p, "DiffBn", DAVIS346_CONFIG_BIAS_DIFFBN, 4, 39);
  addCFBnDavis(p, "ONBn", DAVIS346_CONFIG_BIAS_ONBN, 6, 200);
  addCFBnDavis(p, "OFFBn", DAVIS346_CONFIG_BIAS_OFFBN, 4, 0);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_PIXINVBN, 5, 129);
  addCFBpDavis(p, "PrBp", DAVIS346_CONFIG_BIAS_PRBP, 2, 58);
  addCFBpDavis(p, "PrSFBp", DAVIS346_CONFIG_BIAS_PRSFBP, 1, 33);
  addCFBpDavis(p, "RefrBp", DAVIS346_CONFIG_BIAS_REFRBP, 4, 25);
  addCFBpDavis(p, "", DAVIS346_CONFIG_BIAS_READOUTBUFBP, 6, 20);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_APSROSFBN, 6, 219);
  addCFBpDavis(p, "", DAVIS346_CONFIG_BIAS_ADCCOMPBP, 5, 20);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_COLSELLOWBN, 0, 1);
  addCFBpDavis(p, "", DAVIS346_CONFIG_BIAS_DACBUFBP, 6, 60);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_LCOLTIMEOUTBN, 5, 49);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_AEPDBN, 6, 91);
  addCFBpDavis(p, "", DAVIS346_CONFIG_BIAS_AEPUXBP, 4, 80);
  addCFBpDavis(p, "", DAVIS346_CONFIG_BIAS_AEPUYBP, 7, 152);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_IFREFRBN, 5, 255);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_IFTHRBN, 5, 255);
  addCFBnDavis(p, "", DAVIS346_CONFIG_BIAS_BIASBUFFER, 5, 254);
  addSS(p, "", DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSP, 1, 33, SHIFTED_SOURCE);
  addSS(p, "", DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSN, 1, 33, SHIFTED_SOURCE);
  addBoolDVS(p, "background_activity_filter_enabled", DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, 1);
  addIntDVS(p, "background_activity_filter_time", DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME, 80, 0, 400);
  addBoolDVS(p, "refractory_period_enabled", DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, false);
  addIntDVS(p, "refractory_period_time", DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME, 2, 0, 20);
  addIntDVS(p, "roi_start_col", DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN, 0, 0, 345);
  addIntDVS(p, "roi_start_row", DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, 0, 0, 259);
  addIntDVS(p, "roi_end_col", DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN, 345, 0, 345);
  addIntDVS(p, "roi_end_row", DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, 259, 0, 259);
  addBoolDVS(p, "skip_enabled", DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, false);
  addIntDVS(p, "skip_events_every", DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY, 0, 0, 255);
  addBoolDVS(p, "polarity_flatten", DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN, false);
  addBoolDVS(p, "polarity_suppress", DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS, false);
  addBoolDVS(p, "polarity_suppress_type", DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE, false);
  addInt(p, "aps_roi_start_col", DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, 0, 0, 345);
  addInt(p, "aps_roi_start_row", DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, 0, 0, 259);
  addInt(p, "aps_roi_end_col", DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, 345, 0, 345);
  addInt(p, "aps_roi_end_row", DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, 259, 0, 259);
}

std::shared_ptr<Parameters> make_dvxplorer_parameters()
{
  auto sp = std::make_shared<Parameters>();
  auto * p = sp.get();
  // addBool(p, "dvs_enabled", DVX_DVS, DVX_DVS_RUN, true);
  // addInt(p, "bias_sensitivity", DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, 2, 0, 4);
  addBool(p, "polarity_flatten", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_FLATTEN, false);
  addBool(p, "polarity_on_only", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_ON_ONLY, false);
  addBool(p, "polarity_off_only", DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_OFF_ONLY, false);
  addBool(p, "subsample_enabled", DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_ENABLE, false);
  addIntDvXChip(p, "subsample_vertical", DVX_DVS_CHIP_SUBSAMPLE_VERTICAL, 0, 0, 7);
  addIntDvXChip(p, "subsample_horizontal", DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL, 0, 0, 7);
  addBool(p, "roi_enabled", DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_ENABLE, false);
  addIntDvXCrop(p, "roi_start_col", DVX_DVS_CHIP_CROPPER_X_START_ADDRESS, 0, 0, 639);
  addIntDvXCrop(p, "roi_start_row", DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS, 0, 0, 479);
  addIntDvXCrop(p, "roi_end_col", DVX_DVS_CHIP_CROPPER_X_END_ADDRESS, 0, 0, 639);
  addIntDvXCrop(p, "roi_end_row", DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS, 0, 0, 479);
  addBool(p, "imu_accel_enabled", DVX_DVS_CHIP, DVX_IMU_RUN_ACCELEROMETER, true);
  addBool(p, "imu_gyro_enabled", DVX_DVS_CHIP, DVX_IMU_RUN_GYROSCOPE, true);
  addIntDvXImu(p, "imu_acc_scale", DVX_IMU_ACCEL_RANGE, 1, 0, 3);
  addIntDvXImu(p, "imu_gyro_scale", DVX_IMU_GYRO_RANGE, 2, 0, 4);
  return (sp);
};

static std::shared_ptr<Parameters> make_davis_parameters(int16_t chipID)
{
  auto p = std::make_shared<Parameters>();
  make_davis_common_parameters(p.get());
  switch (chipID) {
    case DAVIS_CHIP_DAVIS346B:
      make_davis346b_parameters(p.get());
      break;
    case DAVIS_CHIP_DAVIS240C:
      make_davis240c_parameters(p.get());
      break;
    default:
      RCLCPP_ERROR_STREAM(logger(), "unknown chip id: " << chipID);
      throw(std::runtime_error("unknown chip id"));
  }
  return (p);
}

std::shared_ptr<Parameters> Parameter::instanceOfParameters(int16_t deviceType, int16_t chipID)
{
  if (parameters) {
    return (parameters);
  }
  switch (deviceType) {
    case CAER_DEVICE_DAVIS:
      parameters = make_davis_parameters(chipID);
      break;
    case CAER_DEVICE_DVXPLORER:
      parameters = make_dvxplorer_parameters();
      break;
    default:
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver"), "unknown device of type: " << deviceType);
      throw(std::runtime_error("unknown device type!"));
      break;
  }
  return (parameters);
}
}  // namespace libcaer_driver
