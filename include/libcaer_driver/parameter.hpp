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

#ifndef LIBCAER_DRIVER__PARAMETER_HPP_
#define LIBCAER_DRIVER__PARAMETER_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace libcaer_driver
{
enum CaerParameterType { INTEGER, BOOLEAN, CF_BIAS, VDAC_BIAS, SHIFTED_SOURCE_BIAS};
enum RosParameterType { ROS_INTEGER, ROS_BOOLEAN };

union PVal {
  int32_t i;
  bool b;
  double f;
};

// need separate namespace because C++ does not allow member function specializations
namespace detail
{
template <class T>
T get(const PVal &);

template <>
inline int32_t get<int32_t>(const PVal & u)
{
  return (u.i);
}
template <>
inline bool get<bool>(const PVal & u)
{
  return (u.b);
}
template <>
inline double get<double>(const PVal & u)
{
  return (u.f);
}
}  // namespace detail

struct Value
{
  explicit Value(int32_t i) { v.i = i; }
  explicit Value(bool b) { v.b = b; }
  explicit Value(double f) { v.f = f; }
  PVal v;
  void operator=(const int & i) { v.i = i; }
  void operator=(const bool & b) { v.b = b; }
  void operator=(const double & f) { v.f = f; }
  template <class T>
  T get() const
  {
    return detail::get<T>(v);
  }
};

struct ValueWithLimits
{
  ValueWithLimits(Value cur, Value nv, Value xv) : curVal(cur), minVal(nv), maxVal(xv) {}
  Value curVal{0};
  Value minVal{0};
  Value maxVal{0};
};

struct RosParameter
{
  explicit RosParameter(
    const std::string & n, RosParameterType t, const ValueWithLimits & v,
    const std::string & d = std::string(""))
  : name(n), type(t), value(v), desc(d)
  {
  }
  std::string name;
  RosParameterType type{ROS_INTEGER};
  ValueWithLimits value;
  std::string desc;
};

class Parameter
{
public:
  using Parameters = std::vector<std::shared_ptr<Parameter>>;
  explicit Parameter(
    const CaerParameterType & t, const std::string &n, int8_t ma, uint8_t pa, bool rb = true)
  : caerType_(t), name_(n), modAddr_(ma), paramAddr_(pa), readBack_(rb)
  {
  }
  virtual ~Parameter() {}
  // ------------- to be overridden by the derived classes
  virtual Value getValue(const std::string & name) const = 0;
  virtual std::vector<RosParameter> getRosParameters() const = 0;
  // -------------
  bool isHidden() const { return (name_.empty());}
  const std::string &getName() const { return (name_); }
  const std::string &getDescription() const { return (description_); }
  CaerParameterType getCaerType() const { return (caerType_); }
  int8_t getModAddr() const { return (modAddr_); }
  uint8_t getParamAddr() const { return (paramAddr_); }
  bool readBack() const { return (readBack_); }
  static std::shared_ptr<Parameters> instanceOfParameters(int16_t deviceType, int16_t chipID);
protected:
  CaerParameterType caerType_{CaerParameterType::INTEGER};
  std::string name_;
  std::string description_;
  int8_t modAddr_{0};
  uint8_t paramAddr_{0};
  bool readBack_{true}; // this parameter can be read back
};

using Parameters = Parameter::Parameters;

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__PARAMETER_HPP_
