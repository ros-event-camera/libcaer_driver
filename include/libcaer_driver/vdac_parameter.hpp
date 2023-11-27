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

#ifndef LIBCAER_DRIVER__VDAC_PARAMETER_HPP_
#define LIBCAER_DRIVER__VDAC_PARAMETER_HPP_

#include <libcaer_driver/parameter.hpp>

namespace libcaer_driver
{
class VDACParameter : public Parameter
{
public:
  explicit VDACParameter(const std::string &name,
    int8_t ma, uint8_t pa, uint8_t vv, uint8_t cv, uint8_t cvn = 0, uint8_t cvx = 7,
    uint8_t vvn = 0, uint8_t vvx = 255)
  : Parameter(CaerParameterType::VDAC_BIAS, name, ma, pa),
    currValue(Value(cv), Value(cvn), Value(cvx)),
    voltValue(Value(vv), Value(vvn), Value(vvx)) {}

  // ------- inherited methods
  std::vector<RosParameter> getRosParameters() const override
  {
    std::vector<RosParameter> p;
    if (!isHidden()) {
      p.push_back(RosParameter(name_ + "_curr", ROS_INTEGER, currValue));
      p.push_back(RosParameter(name_ + "_volt", ROS_INTEGER, voltValue));
    }
    return (p);
  }

  Value getValue(const std::string & name) const override
  {
    return (getValueWithLimits(name).curVal);
  }

  const caer_bias_vdac & getBias() const { return (bias_); }
  void setBias(const std::string & name, int32_t b)
  {
    if (isCurrentBias(name)) {
      bias_.currentValue = b;
      currValue.curVal = Value(b);
    } else {
      bias_.voltageValue = b;
      voltValue.curVal = Value(b);
    }
  }

  ValueWithLimits & getValueWithLimits(const std::string & name)
  {
    return (isCurrentBias(name) ? currValue : voltValue);
  }
  const ValueWithLimits & getValueWithLimits(const std::string & name) const
  {
    return (isCurrentBias(name) ? currValue : voltValue);
  }
  int8_t getModAddr() const { return (modAddr_); }
  uint8_t getParamAddr() const { return (paramAddr_); }
  bool isCurrentBias(const std::string & name) const
  {
    return (name.find("_volt") == std::string::npos);
  }

private:
  caer_bias_vdac bias_;
  ValueWithLimits currValue{Value(0), Value(0), Value(7)};
  ValueWithLimits voltValue{Value(0), Value(0), Value(255)};
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__VDAC_PARAMETER_HPP_
