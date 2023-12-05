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

#include <libcaer_driver/parameter/parameter.hpp>
#include <libcaer_driver/parameter/ros_parameter.hpp>

namespace libcaer_driver
{
class IntegerParameter : public Parameter
{
public:
  explicit IntegerParameter(
    const std::string & name, int8_t ma, uint8_t pa, int32_t v, int32_t vMin, int32_t vMax,
    bool rb = true)
  : Parameter(CaerParameterType::INTEGER, name, ma, pa, rb), v_(v), vMin_(vMin), vMax_(vMax)
  {
  }
  // ------- inherited methods
  std::vector<std::shared_ptr<RosParameter>> makeRosParameters(const std::shared_ptr<Parameter> & pa) const override
  {
    std::vector<std::shared_ptr<RosParameter>> p;
    if (!isHidden()) {
      p.push_back(
        std::make_shared<RosIntParameter>(name_, v_, vMin_, vMax_, description_, pa, FIELD_INT));
    }
    return (p);
  }
  int32_t getValue(Field) const override { return (v_); }
  void setValue(Field, int32_t v) override { v_ = v; }
  // --------- own methods
  int32_t getValue() const { return (v_); }
  int32_t getMinValue() const { return (vMin_); }
  int32_t getMaxValue() const { return (vMax_); }

  void setValue(int32_t v) { v_ = v; }

private:
  int32_t v_{0};
  int32_t vMin_{0};
  int32_t vMax_{0};
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__INTEGER_PARAMETER_HPP_
