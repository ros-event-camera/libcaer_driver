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

#ifndef LIBCAER_DRIVER__CALLBACK_HANDLER_HPP_
#define LIBCAER_DRIVER__CALLBACK_HANDLER_HPP_

#include <libcaer_driver/parameter/parameter.hpp>
#include <libcaercpp/events/frame.hpp>
#include <libcaercpp/events/imu6.hpp>
#include <libcaercpp/events/polarity.hpp>

namespace libcaer_driver
{
class CallbackHandler
{
public:
  CallbackHandler() {}
  virtual ~CallbackHandler() {}
  virtual void declareParameterCallback(const std::shared_ptr<RosParameter> & rp) = 0;
  virtual void deviceDisconnectedCallback() = 0;
  virtual void polarityPacketCallback(
    uint64_t t, const libcaer::events::PolarityEventPacket & packet) = 0;
  virtual void framePacketCallback(
    uint64_t t, const libcaer::events::FrameEventPacket & packet) = 0;
  virtual void imu6PacketCallback(uint64_t t, const libcaer::events::IMU6EventPacket & packet) = 0;
};
}  // namespace libcaer_driver
#endif  // LIBCAER_DRIVER__CALLBACK_HANDLER_HPP_
