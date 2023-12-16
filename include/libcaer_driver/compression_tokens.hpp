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

#ifndef LIBCAER_DRIVER__COMPRESSION_TOKENS_HPP_
#define LIBCAER_DRIVER__COMPRESSION_TOKENS_HPP_

#include <cstdint>

namespace libcaer_driver
{
enum Code { TIME_HIGH, TIME_LOW, ADDR_X, ADDR_Y, VECT_BASE_Y, VECT_8 };

struct __attribute__((packed)) TimeLow
{
  TimeLow(uint16_t ts_usec) : t(ts_usec), code(Code::TIME_LOW) {}
  uint16_t t : 12;
  uint16_t code : 4;
};

struct __attribute__((packed)) TimeHigh
{
  TimeHigh(uint16_t ts) : t(ts), code(Code::TIME_HIGH) {}
  uint16_t t : 12;
  uint16_t code : 4;
};

struct __attribute__((packed)) AddrX
{
  AddrX(uint16_t xa) : x(xa), code(Code::ADDR_X) {}
  uint16_t x : 11;
  uint16_t unused : 1;
  uint16_t code : 4;
};

struct __attribute__((packed)) AddrY
{
  AddrY(uint16_t ya, uint8_t p) : y(ya), polarity(p), code(Code::ADDR_Y) {}
  uint16_t y : 11;
  uint16_t polarity : 1;
  uint16_t code : 4;
};

struct __attribute__((packed)) VectBaseY
{
  VectBaseY(uint16_t ya, uint16_t pa) : y(ya), polarity(pa), code(Code::VECT_BASE_Y) {}
  uint16_t y : 11;
  uint16_t polarity : 1;
  uint16_t code : 4;
};

struct __attribute__((packed)) Vect8
{
  Vect8(uint16_t v) : valid(v), code(Code::VECT_8) {}
  uint16_t valid : 8;
  uint16_t unused : 4;
  uint16_t code : 4;
};

}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__COMPRESSION_TOKENS_HPP_
