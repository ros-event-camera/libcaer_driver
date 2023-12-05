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

#ifndef LIBCAER_DRIVER__PARAMETER__PARAMETER_HPP_
#define LIBCAER_DRIVER__PARAMETER__PARAMETER_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace libcaer_driver
{
enum CaerParameterType { INTEGER, BOOLEAN, CF_BIAS, VDAC_BIAS, SHIFTED_SOURCE_BIAS };
enum Field {
  FIELD_INT,
  FIELD_BOOL,
  FIELD_COARSE,
  FIELD_FINE,
  FIELD_VOLTAGE,
  FIELD_CURRENT,
  FIELD_REF_VALUE,
  FIELD_REG_VALUE
};

class RosParameter;  // forward decl

class Parameter
{
public:
  struct ParameterVector : public std::vector<std::shared_ptr<Parameter>>
  {
    inline void add(const value_type & v) { push_back(v); }
  };

  using Parameters = ParameterVector;

  explicit Parameter(
    const CaerParameterType & t, const std::string & n, int8_t ma, uint8_t pa, bool rb = true)
  : caerType_(t), name_(n), modAddr_(ma), paramAddr_(pa), readBack_(rb)
  {
  }
  virtual ~Parameter() {}
  // ------------- to be overridden by the derived classes
  virtual std::vector<std::shared_ptr<RosParameter>> makeRosParameters(
    const std::shared_ptr<Parameter> & pa) const = 0;
  virtual int32_t getValue(Field f) const = 0;
  virtual void setValue(Field f, int32_t v) = 0;
  // -------------
  bool isHidden() const { return (name_.empty()); }
  const std::string & getName() const { return (name_); }
  const std::string & getDescription() const { return (description_); }
  CaerParameterType getCaerType() const { return (caerType_); }
  int8_t getModAddr() const { return (modAddr_); }
  uint8_t getParamAddr() const { return (paramAddr_); }
  bool readBack() const { return (readBack_); }

protected:
  CaerParameterType caerType_{CaerParameterType::INTEGER};
  std::string name_;
  std::string description_;
  int8_t modAddr_{0};
  uint8_t paramAddr_{0};
  bool readBack_{true};  // this parameter should be read back
};

using Parameters = Parameter::Parameters;

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__PARAMETER__PARAMETER_HPP_
