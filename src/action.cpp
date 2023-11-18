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

#include <libcaer_driver/action.hpp>
#include <libcaercpp/devices/device_discover.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>

//
// Map between action (e.g. reset time stamp) and corresponding
// libcaer parameters for configSet().

namespace libcaer_driver
{
namespace action
{
using ActionMap = std::map<std::string, Action>;
static ActionMap davisActions{{
  {"time_reset", {DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 1}},
}};

static ActionMap dvXplorerActions{{
  {"time_reset", {DVX_MUX, DVX_MUX_TIMESTAMP_RESET, 1}},
}};

static const std::map<int, ActionMap> actionMaps{
  {{CAER_DEVICE_DVXPLORER, dvXplorerActions}, {CAER_DEVICE_DAVIS, davisActions}}};

Action get(int deviceType, const std::string & name)
{
  const auto map_it = actionMaps.find(deviceType);
  if (map_it == actionMaps.end()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("driver"), "cannot find action map for device type " << deviceType);
    throw(std::runtime_error("no action map for device type"));
  }
  const auto & ac_it = map_it->second.find(name);
  if (ac_it == map_it->second.end()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver"), "cannot find action " << name);
    throw(std::runtime_error("cannot find action"));
  }
  return (ac_it->second);
}
}  // namespace action

}  // namespace libcaer_driver
