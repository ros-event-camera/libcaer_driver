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

#ifndef LIBCAER_DRIVER__SHIFTED_SOURCE_PARAMETER_HPP_
#define LIBCAER_DRIVER__SHIFTED_SOURCE_PARAMETER_HPP_

#include <libcaer_driver/parameter.hpp>

namespace libcaer_driver
{
class ShiftedSourceParameter : public Parameter
{
public:
  explicit ShiftedSourceParameter(
    const std::string & name, int8_t ma, uint8_t pa, uint8_t ref, uint8_t reg, uint8_t om)
  : Parameter(CaerParameterType::SHIFTED_SOURCE_BIAS, name, ma, pa)
  {
    shiftedSource_.refValue = ref;
    shiftedSource_.regValue = reg;
    shiftedSource_.operatingMode = static_cast<caer_bias_shiftedsource_operating_mode>(om);
    shiftedSource_.voltageLevel = SPLIT_GATE;
  }

  // ------- inherited methods
  std::vector<RosParameter> getRosParameters() const override
  {
    return (std::vector<RosParameter>());
  }

  Value getValue(const std::string &) const override { return (Value(0)); }

private:
  caer_bias_shiftedsource shiftedSource_;
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__SHIFTED_SOURCE_PARAMETER_HPP_
