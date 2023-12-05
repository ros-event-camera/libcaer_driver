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

#ifndef LIBCAER_DRIVER__PARAMETER__BOOLEAN_PARAMETER_HPP_
#define LIBCAER_DRIVER__PARAMETER__BOOLEAN_PARAMETER_HPP_

#include <libcaer_driver/parameter/parameter.hpp>
#include <libcaer_driver/parameter/ros_parameter.hpp>

namespace libcaer_driver
{
class BooleanParameter : public Parameter
{
public:
  explicit BooleanParameter(
    const std::string & name, int8_t ma, uint8_t pa, bool v, bool rb = false)
  : Parameter(CaerParameterType::BOOLEAN, name, ma, pa, rb), value_(v)
  {
  }
  // ------- inherited methods
  std::vector<std::shared_ptr<RosParameter>> makeRosParameters(
    const std::shared_ptr<Parameter> & pa) const override
  {
    std::vector<std::shared_ptr<RosParameter>> p;
    if (!isHidden()) {
      p.push_back(std::make_shared<RosBoolParameter>(name_, value_, description_, pa, FIELD_BOOL));
    }
    return (p);
  }
  int32_t getValue(Field) const override { return (static_cast<int32_t>(value_)); }
  void setValue(Field, int32_t v) override { value_ = static_cast<bool>(v); }
  // ------- own methods

  bool getValue() { return (value_); }
  void setValue(bool b) { value_ = b; }

private:
  bool value_;
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__PARAMETER__BOOLEAN_PARAMETER_HPP_
