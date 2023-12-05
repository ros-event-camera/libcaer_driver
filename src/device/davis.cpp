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
#include <libcaer_driver/logging.hpp>
#include <libcaer_driver/parameter/boolean_parameter.hpp>
#include <libcaer_driver/parameter/coarse_fine_parameter.hpp>
#include <libcaer_driver/parameter/integer_parameter.hpp>
#include <libcaer_driver/parameter/shifted_source_parameter.hpp>
#include <libcaer_driver/parameter/vdac_parameter.hpp>

namespace libcaer_driver
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("device")); }
//
// The following definitions are used for a more compact notation when building the
// parameter list.

static auto Bool = [](const std::string & n, int8_t ma, uint8_t pa, bool def) {
  return (std::make_shared<BooleanParameter>(n, ma, pa, def));
};
static auto BoolDVS = [](const std::string & n, uint8_t pa, bool def) {
  return (std::make_shared<BooleanParameter>(n, DAVIS_CONFIG_DVS, pa, def));
};

static auto IntDVS = [](const std::string & n, uint8_t pa, int32_t def, int32_t vn, int32_t vx) {
  return (std::make_shared<IntegerParameter>(n, DAVIS_CONFIG_DVS, pa, def, vn, vx, true));
};
static auto IntAPS = [](const std::string & n, uint8_t pa, int32_t def, int32_t vn, int32_t vx) {
  return (std::make_shared<IntegerParameter>(n, DAVIS_CONFIG_APS, pa, def, vn, vx, true));
};
static auto IntIMU = [](const std::string & n, uint8_t pa, int32_t def, int32_t vn, int32_t vx) {
  return (std::make_shared<IntegerParameter>(n, DAVIS_CONFIG_IMU, pa, def, vn, vx, true));
};

static auto VDAC = [](const std::string & n, uint8_t pa, uint8_t defV, uint8_t defC) {
  return (std::make_shared<VDACParameter>(n, DAVIS_CONFIG_BIAS, pa, defV, defC));
};
static auto BiasN = [](const std::string & n, uint8_t pa, uint8_t defC, uint8_t defF) {
  return (std::make_shared<CoarseFineParameter>(n, DAVIS_CONFIG_BIAS, pa, defC, defF, true));
};
static auto BiasP = [](const std::string & n, uint8_t pa, uint8_t defC, uint8_t defF) {
  return (std::make_shared<CoarseFineParameter>(n, DAVIS_CONFIG_BIAS, pa, defC, defF, false));
};
static auto BiasSS = [](const std::string & n, uint8_t pa, uint8_t ref, uint8_t reg, uint8_t om) {
  return (std::make_shared<ShiftedSourceParameter>(n, DAVIS_CONFIG_BIAS, pa, ref, reg, om));
};

static void make_davis_common_parameters(Parameters * p)
{
  // ----------- IMU
  p->add(Bool("imu_acc_enabled", DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, true));
  p->add(Bool("imu_gyro_enabled", DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, true));
  p->add(Bool("imu_temp_enabled", DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, true));
  p->add(IntIMU("imu_acc_scale", DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, 3, 0, 3));
  p->add(IntIMU("imu_gyro_scale", DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, 3, 0, 3));
  p->add(IntIMU("imu_low_pass_filter", DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER, 1, 0, 6));
  // note: set rate divider to 7 when lowpass filter is enabled (!=0), otherwise set to 0
  p->add(IntIMU("imu_sample_rate_divider", DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 7, 0, 7));
  // ----------- APS
  p->add(Bool("aps_enabled", DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, true));
  p->add(IntAPS("aps_exposure", DAVIS_CONFIG_APS_EXPOSURE, 5000, 0, 1000000));
  p->add(IntAPS("aps_frame_mode", DAVIS_CONFIG_APS_FRAME_MODE, 0, 0, 2));
  p->add(IntAPS("aps_frame_interval", DAVIS_CONFIG_APS_FRAME_INTERVAL, 25000, 0, 8388607));
  // ---------- DVS
  p->add(Bool("dvs_enabled", DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, true));
};

static void make_davis240c_parameters(Parameters * p)
{
  p->add(BiasP("PrBp", DAVIS240_CONFIG_BIAS_PRBP, 2, 58));
  p->add(BiasP("PrSFBp", DAVIS240_CONFIG_BIAS_PRSFBP, 1, 33));
  p->add(BiasN("DiffBn", DAVIS240_CONFIG_BIAS_DIFFBN, 4, 39));
  p->add(BiasN("ONBn", DAVIS240_CONFIG_BIAS_ONBN, 6, 200));
  p->add(BiasN("OFFBn", DAVIS240_CONFIG_BIAS_OFFBN, 4, 0));
  p->add(BiasP("RefrBp", DAVIS240_CONFIG_BIAS_REFRBP, 4, 25));
}

static void make_davis346b_parameters(Parameters * p)
{
  p->add(VDAC("", DAVIS346_CONFIG_BIAS_APSOVERFLOWLEVEL, 27, 6));
  p->add(VDAC("", DAVIS346_CONFIG_BIAS_APSCAS, 21, 6));
  p->add(VDAC("ADC_RefHigh", DAVIS346_CONFIG_BIAS_ADCREFHIGH, 27, 7));
  p->add(VDAC("ADC_RefLow", DAVIS346_CONFIG_BIAS_ADCREFLOW, 1, 7));
  p->add(VDAC("", DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE, 21, 7));
  p->add(VDAC("", DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE, 21, 7));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_LOCALBUFBN, 5, 164));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_PADFOLLBN, 7, 215));
  p->add(BiasN("DiffBn", DAVIS346_CONFIG_BIAS_DIFFBN, 4, 39));
  p->add(BiasN("ONBn", DAVIS346_CONFIG_BIAS_ONBN, 6, 200));
  p->add(BiasN("OFFBn", DAVIS346_CONFIG_BIAS_OFFBN, 4, 0));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_PIXINVBN, 5, 129));
  p->add(BiasP("PrBp", DAVIS346_CONFIG_BIAS_PRBP, 2, 58));
  p->add(BiasP("PrSFBp", DAVIS346_CONFIG_BIAS_PRSFBP, 1, 33));
  p->add(BiasP("RefrBp", DAVIS346_CONFIG_BIAS_REFRBP, 4, 25));
  p->add(BiasP("", DAVIS346_CONFIG_BIAS_READOUTBUFBP, 6, 20));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_APSROSFBN, 6, 219));
  p->add(BiasP("", DAVIS346_CONFIG_BIAS_ADCCOMPBP, 5, 20));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_COLSELLOWBN, 0, 1));
  p->add(BiasP("", DAVIS346_CONFIG_BIAS_DACBUFBP, 6, 60));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_LCOLTIMEOUTBN, 5, 49));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_AEPDBN, 6, 91));
  p->add(BiasP("", DAVIS346_CONFIG_BIAS_AEPUXBP, 4, 80));
  p->add(BiasP("", DAVIS346_CONFIG_BIAS_AEPUYBP, 7, 152));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_IFREFRBN, 5, 255));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_IFTHRBN, 5, 255));
  p->add(BiasN("", DAVIS346_CONFIG_BIAS_BIASBUFFER, 5, 254));
  p->add(BiasSS("", DAVIS346_CONFIG_BIAS_SSP, 1, 33, SHIFTED_SOURCE));
  p->add(BiasSS("", DAVIS346_CONFIG_BIAS_SSN, 1, 33, SHIFTED_SOURCE));
  p->add(
    BoolDVS("background_activity_filter_enabled", DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, 1));
  p->add(IntDVS(
    "background_activity_filter_time", DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME, 80, 0,
    400));
  p->add(BoolDVS("refractory_period_enabled", DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, false));
  p->add(
    IntDVS("refractory_period_time", DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME, 2, 0, 20));
  p->add(IntDVS("roi_start_col", DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN, 0, 0, 345));
  p->add(IntDVS("roi_start_row", DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, 0, 0, 259));
  p->add(IntDVS("roi_end_col", DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN, 345, 0, 345));
  p->add(IntDVS("roi_end_row", DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, 259, 0, 259));
  p->add(BoolDVS("skip_enabled", DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, false));
  p->add(IntDVS("skip_events_every", DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY, 0, 0, 255));
  p->add(BoolDVS("polarity_flatten", DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN, false));
  p->add(BoolDVS("polarity_suppress", DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS, false));
  p->add(BoolDVS("polarity_suppress_type", DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE, false));
  p->add(IntAPS("aps_roi_start_col", DAVIS_CONFIG_APS_START_COLUMN_0, 0, 0, 345));
  p->add(IntAPS("aps_roi_start_row", DAVIS_CONFIG_APS_START_ROW_0, 0, 0, 259));
  p->add(IntAPS("aps_roi_end_col", DAVIS_CONFIG_APS_END_COLUMN_0, 345, 0, 345));
  p->add(IntAPS("aps_roi_end_row", DAVIS_CONFIG_APS_END_ROW_0, 259, 0, 259));
}

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
      BOMB_OUT("unknown chip id: " << chipID);
  }
  return (p);
}

Davis::Davis(int16_t chipID) { parameters_ = make_davis_parameters(chipID); }

void Davis::resetTimeStamps()
{
  device_->configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 1);
}

}  // namespace libcaer_driver
