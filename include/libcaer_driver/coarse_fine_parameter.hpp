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

#ifndef LIBCAER_DRIVER__COARSE_FINE_PARAMETER_HPP_
#define LIBCAER_DRIVER__COARSE_FINE_PARAMETER_HPP_

#include <libcaer/devices/davis.h>
#include <libcaer_driver/parameter.hpp>

namespace libcaer_driver
{
class CoarseFineParameter : public Parameter
{
public:
  explicit CoarseFineParameter(const std::string &name,
    int8_t ma, uint8_t pa, uint8_t cv, uint8_t fv, bool sexN, uint8_t cvn = 0, uint8_t cvx = 7,
    uint8_t fvn = 0, uint8_t fvx = 255)
  : Parameter(CaerParameterType::CF_BIAS, name, ma, pa),
    coarseValue(Value(cv), Value(cvn), Value(cvx)),
    fineValue(Value(fv), Value(fvn), Value(fvx)), sexN_(sexN)
  {
  }

  // ------- inherited methods
  std::vector<RosParameter> getRosParameters() const override
  {
    std::vector<RosParameter> p;
    if (!isHidden()) {
      p.push_back(RosParameter(name_ + "_coarse", ROS_INTEGER, coarseValue));
      p.push_back(RosParameter(name_ + "_fine", ROS_INTEGER, fineValue));
    }
    return (p);
  }

  Value getValue(const std::string & name) const override
  {
    return (getValueWithLimits(name).curVal);
  }

  const caer_bias_coarsefine & getBias() const { return (bias_); }
  void setBias(const std::string & name, int32_t b)
  {
    if (isCoarseBias(name)) {
      bias_.coarseValue = b;
      coarseValue.curVal = Value(b);
    } else {
      bias_.fineValue = b;
      fineValue.curVal = Value(b);
    }
    bias_.sexN = sexN_;
  }

  ValueWithLimits & getValueWithLimits(const std::string & name)
  {
    return (isCoarseBias(name) ? coarseValue : fineValue);
  }
  const ValueWithLimits & getValueWithLimits(const std::string & name) const
  {
    return (isCoarseBias(name) ? coarseValue : fineValue);
  }
  bool isCoarseBias(const std::string & name) const
  {
    return (name.find("_coarse") != std::string::npos);
  }

private:
  caer_bias_coarsefine bias_;
  ValueWithLimits coarseValue{Value(0), Value(0), Value(7)};
  ValueWithLimits fineValue{Value(0), Value(0), Value(255)};
  bool sexN_{false};
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__COARSE_FINE_PARAMETER_HPP_
