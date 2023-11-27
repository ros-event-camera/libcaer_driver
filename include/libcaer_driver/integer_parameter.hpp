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

#ifndef LIBCAER_DRIVER__INTEGER_PARAMETER_HPP_
#define LIBCAER_DRIVER__INTEGER_PARAMETER_HPP_

#include <libcaer_driver/parameter.hpp>

namespace libcaer_driver
{
class IntegerParameter : public Parameter
{
public:
  explicit IntegerParameter(const std::string &name,
    int8_t ma, uint8_t pa, int32_t v, int32_t vn, int32_t vx)
  : Parameter(CaerParameterType::INTEGER, name, ma, pa),
    valueWithLimits_(Value(v), Value(vn), Value(vx))
  {
  }
  // ------- inherited methods
  std::vector<RosParameter> getRosParameters() const override
  {
    std::vector<RosParameter> p;
    if (!isHidden()) {
      p.push_back(RosParameter(name_, ROS_INTEGER, valueWithLimits_));
    }
    return (p);
  }

  Value getValue(const std::string &) const override
  {
    return (valueWithLimits_.curVal);
  }
  // ----------------------------

  void setValue(int32_t b) {
    valueWithLimits_.curVal = Value(b);
  }

  ValueWithLimits & getValueWithLimits()
  {
    return (valueWithLimits_);
  }
  const ValueWithLimits & getValueWithLimits() const
  {
    return (valueWithLimits_);
  }

private:
  ValueWithLimits valueWithLimits_;
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__INTEGER_PARAMETER_HPP_
