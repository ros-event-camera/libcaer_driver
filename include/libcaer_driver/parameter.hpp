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

namespace libcaer_driver
{
enum RosParamType { INVALID, INTEGER, BOOLEAN, DOUBLE };

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

struct Parameter
{
  Parameter(RosParamType t, uint8_t ma, uint8_t pa, bool s, uint8_t def, int32_t nV, int32_t xV)
  : type(t), modAddr(ma), paramAddr(pa), sexN(s)
  {
    defVal = def;
    minVal = nV;
    maxVal = xV;
  }
  Parameter(RosParamType t, uint8_t ma, uint8_t pa, bool def)
  : type(BOOLEAN), modAddr(ma), paramAddr(pa)
  {
    (void)t;
    defVal = def;
  }
  // ------------- variables ------------------
  RosParamType type{INVALID};
  int8_t modAddr{0};
  uint8_t paramAddr{0};
  bool sexN{true};
  Value defVal{0};  // default
  Value minVal{0};  // min
  Value maxVal{0};  // max
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__PARAMETER_HPP_
