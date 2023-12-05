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

#ifndef LIBCAER_DRIVER__PARAMETER__VDAC_PARAMETER_HPP_
#define LIBCAER_DRIVER__PARAMETER__VDAC_PARAMETER_HPP_

#include <libcaer_driver/parameter/parameter.hpp>

namespace libcaer_driver
{
class VDACParameter : public Parameter
{
public:
  explicit VDACParameter(
    const std::string & name, int8_t ma, uint8_t pa, uint8_t v, uint8_t c, uint8_t vn = 0,
    uint8_t vx = 63, uint8_t cn = 0, uint8_t cx = 7)
  : Parameter(CaerParameterType::VDAC_BIAS, name, ma, pa),
    voltMin_(vn),
    voltMax_(vx),
    currMin_(cn),
    currMax_(cx)
  {
    bias_.voltageValue = v;
    bias_.currentValue = c;
  }

  // ----------- inherited methods
  std::vector<std::shared_ptr<RosParameter>> makeRosParameters(
    const std::shared_ptr<Parameter> & pa) const override
  {
    std::vector<std::shared_ptr<RosParameter>> p;
    if (!isHidden()) {
      p.push_back(std::make_shared<RosIntParameter>(
        name_ + "_voltage", bias_.voltageValue, voltMin_, voltMax_, description_, pa,
        FIELD_VOLTAGE));
      p.push_back(std::make_shared<RosIntParameter>(
        name_ + "_current", bias_.currentValue, currMin_, currMax_, description_, pa,
        FIELD_CURRENT));
    }
    return (p);
  }

  int32_t getValue(Field f) const override
  {
    return ((f == FIELD_VOLTAGE) ? bias_.voltageValue : bias_.currentValue);
  }
  void setValue(Field f, int32_t v) override { setBias(f, v); }
  // ---------------------------------------
  const caer_bias_vdac & getBias() const { return (bias_); }

  void setBias(Field f, int32_t b)
  {
    if (f == FIELD_CURRENT) {
      bias_.currentValue = b;
    } else {
      bias_.voltageValue = b;
    }
  }
  void setBias(const caer_bias_vdac & b) { bias_ = b; }

private:
  caer_bias_vdac bias_;
  uint8_t voltMin_{0};
  uint8_t voltMax_{0};
  uint8_t currMin_{0};
  uint8_t currMax_{0};
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__PARAMETER__VDAC_PARAMETER_HPP_
