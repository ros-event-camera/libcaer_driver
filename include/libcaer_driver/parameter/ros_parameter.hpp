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

#ifndef LIBCAER_DRIVER__PARAMETER__ROS_PARAMETER_HPP_
#define LIBCAER_DRIVER__PARAMETER__ROS_PARAMETER_HPP_

#include <algorithm>  // for std::clamp
#include <cstdint>
#include <libcaer_driver/parameter/parameter.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace libcaer_driver
{
enum RosParameterType { ROS_INVALID, ROS_INT, ROS_BOOL, ROS_FLOAT };

class RosParameter
{
public:
  RosParameter(
    RosParameterType t, const std::string & name, const std::string & desc,
    const std::shared_ptr<Parameter> & p, Field f)
  : type_(t), name_(name), desc_(desc), param_(p), field_(f)
  {
  }
  virtual ~RosParameter() {}
  const RosParameterType & getType() const { return (type_); }
  void setParameter(const std::shared_ptr<Parameter> & p) { param_ = p; }
  const std::string & getName() const { return (name_); }
  const std::string & getDescription() const { return (desc_); }
  const std::shared_ptr<Parameter> & getParameter() const { return (param_); }
  Field getField() const { return (field_); }

protected:
  RosParameterType type_{ROS_INVALID};
  std::string name_;
  std::string desc_;                  // description
  std::shared_ptr<Parameter> param_;  // pointer to libcaer parameter
  Field field_{FIELD_INT};
};

class RosIntParameter : public RosParameter
{
public:
  explicit RosIntParameter(
    const std::string & name, int32_t v, int32_t vMin, int32_t vMax, const std::string & desc,
    const std::shared_ptr<Parameter> & p, Field f)
  : RosParameter(ROS_INT, name, desc, p, f), v_(v), vMin_(vMin), vMax_(vMax)
  {
  }
  int32_t clamp(int32_t v) { return (std::clamp<int32_t>(v, vMin_, vMax_)); }
  int32_t getMinValue() const { return (vMin_); }
  int32_t getMaxValue() const { return (vMax_); }

  int32_t getValue() const { return (v_); }

private:
  int32_t v_{0};
  int32_t vMin_{0};
  int32_t vMax_{0};
};

class RosBoolParameter : public RosParameter
{
public:
  explicit RosBoolParameter(
    const std::string & name, bool v, const std::string & desc,
    const std::shared_ptr<Parameter> & p, Field f)
  : RosParameter(ROS_BOOL, name, desc, p, f), v_(v)
  {
  }

private:
  bool v_{0};
};

class RosFloatParameter : public RosParameter
{
public:
  explicit RosFloatParameter(
    const std::string & name, float v, float vMin, float vMax, const std::string & desc,
    const std::shared_ptr<Parameter> & p, Field f)
  : RosParameter(ROS_FLOAT, name, desc, p, f), v_(v), vMin_(vMin), vMax_(vMax)
  {
  }
  auto clamp(float v) { return (std::clamp<float>(v, vMin_, vMax_)); }
  auto getMinValue() const { return (vMin_); }
  auto getMaxValue() const { return (vMax_); }

  auto getValue() const { return (v_); }

private:
  float v_{0};
  float vMin_{0};
  float vMax_{0};
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__PARAMETER__ROS_PARAMETER_HPP_
