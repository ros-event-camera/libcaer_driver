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

#ifndef LIBCAER_DRIVER__BOOLEAN_PARAMETER_HPP_
#define LIBCAER_DRIVER__BOOLEAN_PARAMETER_HPP_

#include <libcaer_driver/parameter.hpp>

namespace libcaer_driver
{
class BooleanParameter : public Parameter
{
public:
  explicit BooleanParameter(const std::string &name,
    int8_t ma, uint8_t pa, bool v)
  : Parameter(CaerParameterType::BOOLEAN, name, ma, pa),
    value_(Value(v))
  {
  }
  // ------- inherited methods
  std::vector<RosParameter> getRosParameters() const override
  {
    std::vector<RosParameter> p;
    if (!isHidden()) {
      p.push_back(RosParameter(name_, ROS_BOOLEAN, ValueWithLimits(value_, Value(false), Value(true))));
    }
    return (p);
  }

  Value getValue(const std::string &) const override
  {
    return (value_);
  }
  // ----------------------------
  bool getValue() const 
  {
    return (value_.get<bool>());
  }
  
  void setValue(bool b) {
    value_ = Value(b);
  }

private:
  Value value_;
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__BOOLEAN_PARAMETER_HPP_
