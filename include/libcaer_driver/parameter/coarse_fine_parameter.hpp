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

#ifndef LIBCAER_DRIVER__PARAMETER__COARSE_FINE_PARAMETER_HPP_
#define LIBCAER_DRIVER__PARAMETER__COARSE_FINE_PARAMETER_HPP_

#include <libcaer/devices/davis.h>

#include <libcaer_driver/parameter/parameter.hpp>
#include <libcaer_driver/parameter/ros_parameter.hpp>

namespace libcaer_driver
{
class CoarseFineParameter : public Parameter
{
public:
  explicit CoarseFineParameter(
    const std::string & name, int8_t ma, uint8_t pa, uint8_t c, uint8_t f, bool sexN,
    uint8_t cn = 0, uint8_t cx = 7, uint8_t fn = 0, uint8_t fx = 255)
  : Parameter(CaerParameterType::CF_BIAS, name, ma, pa),
    coarseMin_(cn),
    coarseMax_(cx),
    fineMin_(fn),
    fineMax_(fx),
    sexN_(sexN)
  {
    bias_.coarseValue = c;
    bias_.fineValue = f;
    bias_.sexN = sexN_;
    bias_.typeNormal = true;
    bias_.enabled = true;
  }

  // ------- inherited methods
  std::vector<std::shared_ptr<RosParameter>> makeRosParameters(
    const std::shared_ptr<Parameter> & pa) const override
  {
    std::vector<std::shared_ptr<RosParameter>> p;
    if (!isHidden()) {
      p.push_back(std::make_shared<RosIntParameter>(
        name_ + "_coarse", bias_.coarseValue, coarseMin_, coarseMax_, description_, pa,
        FIELD_COARSE));
      p.push_back(std::make_shared<RosIntParameter>(
        name_ + "_fine", bias_.fineValue, fineMin_, fineMax_, description_, pa, FIELD_FINE));
    }
    return (p);
  }
  int32_t getValue(Field f) const override
  {
    return ((f == FIELD_COARSE) ? bias_.coarseValue : bias_.fineValue);
  }
  void setValue(Field f, int32_t v) override { setBias(f, v); }
  // -------------------------

  const caer_bias_coarsefine & getBias() const { return (bias_); }

  void setBias(const caer_bias_coarsefine & b) { bias_ = b; }
  void setBias(Field f, int32_t b)
  {
    if (f == FIELD_COARSE) {
      bias_.coarseValue = b;
    } else {
      bias_.fineValue = b;
    }
    bias_.sexN = sexN_;
  }

private:
  caer_bias_coarsefine bias_;
  uint8_t coarseMin_{0};
  uint8_t coarseMax_{0};
  uint8_t fineMin_{0};
  uint8_t fineMax_{0};
  bool sexN_{false};
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__PARAMETER__COARSE_FINE_PARAMETER_HPP_
