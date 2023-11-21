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

#ifndef LIBCAER_DRIVER__LIBCAER_WRAPPER_HPP_
#define LIBCAER_DRIVER__LIBCAER_WRAPPER_HPP_

#include <libcaer/devices/davis.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <libcaercpp/devices/device.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "libcaer_driver/callback_handler.hpp"
#include "libcaer_driver/parameter.hpp"

namespace libcaer_driver
{
class LibcaerWrapper;  // forward declaration
namespace detail
{
// forward declare main template, specialize in cpp file
template <class T>
T set_parameter(LibcaerWrapper *, const std::string &, const Parameter &, const T)
{
  return (T());
}
template <>
int32_t set_parameter<int32_t>(
  LibcaerWrapper * wrapper, const std::string & name, const Parameter & p, const int32_t value);
template <>
bool set_parameter<bool>(
  LibcaerWrapper * wrapper, const std::string & name, const Parameter & p, const bool value);

}  // namespace detail

class LibcaerWrapper
{
public:
  struct Stats
  {
    size_t msgsSent{0};
    size_t msgsRecv{0};
    size_t bytesSent{0};
    size_t bytesRecv{0};
    size_t maxQueueSize{0};
    size_t eventsRecv{0};
  };

  LibcaerWrapper();
  ~LibcaerWrapper();

  void initialize(const std::string & deviceType, int deviceId, const std::string & restrictSerial);
  void setCallbackHandler(CallbackHandler * cb) { callbackHandler_ = cb; }
  void resetTimeStamps();

  inline void updateMsgsSent(int inc)
  {
    std::unique_lock<std::mutex> lock(statsMutex_);
    stats_.msgsSent += inc;
  }
  inline void updateBytesSent(int inc)
  {
    std::unique_lock<std::mutex> lock(statsMutex_);
    stats_.bytesSent += inc;
  }

  bool startSensor();
  void stopSensor();
  uint32_t getWidth() const { return (static_cast<uint32_t>(width_)); }
  uint32_t getHeight() const { return (static_cast<uint32_t>(height_)); }
  const std::string & getSerialNumber() const { return (serialNumber_); }
  bool isMaster() const { return (isMaster_); }

  void setStatisticsInterval(double sec) { statsInterval_ = sec; }
  void deviceDisconnected();
  const ParameterMap & getParameters();
  int32_t setIntegerParameter(const std::string & name, const Parameter & p, int32_t value);

  template <class T>
  T setParameter(const std::string & name, const Parameter & p, T value)
  {
    return (detail::set_parameter<T>(this, name, p, value));
  }

private:
  void processingThread();
  void statsThread();
  void printStatistics();
  int setCoarseFineBias(const std::string & name, const Parameter & p, int value);
  void startProcessingThread();
  void stopProcessingThread();
  void stopStatsThread();
  void processPacket(uint64_t nsSinceEpoch, const libcaer::events::EventPacket & packet);

  // ------------ variables
  CallbackHandler * callbackHandler_{nullptr};

  int width_{0};   // image width
  int height_{0};  // image height
  std::string serialNumber_;
  bool isMaster_{true};  // whether it is configured as master or not
  std::unique_ptr<libcaer::devices::device> device_;
  int16_t deviceType_{0};
  bool deviceRunning_{false};  // status of device
  std::map<std::string, Parameter> biases_;
  // --  related to statistics
  double statsInterval_{2.0};  // time between printouts (seconds)
  std::chrono::time_point<std::chrono::system_clock> lastPrintTime_;
  Stats stats_;
  std::mutex statsMutex_;
  std::condition_variable statsCv_;
  std::shared_ptr<std::thread> statsThread_;
  std::atomic_bool keepStatsRunning_{false};
  // -- related to processing thread
  std::shared_ptr<std::thread> processingThread_;
  std::atomic_bool keepProcessingRunning_{false};
};

}  // namespace libcaer_driver
#endif  // LIBCAER_DRIVER__LIBCAER_WRAPPER_HPP_
