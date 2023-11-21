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

#ifndef LIBCAER_DRIVER__ACTION_HPP_
#define LIBCAER_DRIVER__ACTION_HPP_

#include <cstdint>
#include <string>
#include <tuple>

namespace libcaer_driver
{
namespace action
{
using Action = std::tuple<int8_t, uint8_t, uint32_t>;
Action get(int deviceType, const std::string & name);
}  // namespace action
}  // namespace libcaer_driver

#endif  // LIBCAER_DRIVER__ACTION_HPP_