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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <libcaercpp/devices/device.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "libcaer_driver/callback_handler.hpp"

namespace libcaer_driver
{
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
  };

  LibcaerWrapper();
  ~LibcaerWrapper();

  void initialize(const std::string & deviceType, int deviceId, const std::string & restrictSerial);

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
  bool stopSensor();
  int getWidth() const { return (width_); }
  int getHeight() const { return (height_); }
  const std::string & getSerialNumber() const { return (serialNumber_); }

  void setDeviceType(const std::string & devType) { deviceType_ = devType; }
  void setStatisticsInterval(double sec) { statsInterval_ = sec; }

private:
  void processingThread();
  void statsThread();
  void printStatistics();
  void initializeSensor();
  void startThreads();
  void stopThreads();

  // ------------ variables
  CallbackHandler * callbackHandler_{0};
  int width_{0};   // image width
  int height_{0};  // image height
  std::string restrictSN_;
  std::string serialNumber_;
  std::string deviceType_;
  int deviceId_{0};
  // --  related to statistics
  double statsInterval_{2.0};  // time between printouts
  std::chrono::time_point<std::chrono::system_clock> lastPrintTime_;
  Stats stats_;
  std::mutex statsMutex_;
  std::shared_ptr<std::thread> statsThread_;
  std::shared_ptr<std::thread> processingThread_;
  std::unique_ptr<libcaer::devices::device> device_;
  std::atomic_bool keepRunning_{true};
};
}  // namespace libcaer_driver
#endif  // LIBCAER_DRIVER__LIBCAER_WRAPPER_HPP_
