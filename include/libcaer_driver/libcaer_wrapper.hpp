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
#include <libcaer_driver/callback_handler.hpp>
#include <libcaer_driver/device/device.hpp>
#include <libcaer_driver/device_info.hpp>
#include <libcaer_driver/parameter/boolean_parameter.hpp>
#include <libcaer_driver/parameter/coarse_fine_parameter.hpp>
#include <libcaer_driver/parameter/integer_parameter.hpp>
#include <libcaer_driver/parameter/parameter.hpp>
#include <libcaer_driver/parameter/shifted_source_parameter.hpp>
#include <libcaer_driver/parameter/vdac_parameter.hpp>
#include <libcaercpp/devices/device.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

// #define DEBUG_PERFORMANCE

namespace libcaer_driver
{
class LibcaerWrapper;  // forward declaration
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
#ifdef DEBUG_PERFORMANCE
    size_t timeElapsed{0};
#endif
  };

  LibcaerWrapper();
  ~LibcaerWrapper();

  bool hasDVS() const { return (getDeviceInfo().hasDVS); }
  bool hasIMU() const { return (getDeviceInfo().hasIMU); }
  bool isMaster() const { return (getDeviceInfo().deviceIsMaster); }
  int16_t getDVSSizeX() const { return (getDeviceInfo().dvsSizeX); }
  int16_t getDVSSizeY() const { return (getDeviceInfo().dvsSizeY); }
  int16_t getAPSSizeX() const { return (getDeviceInfo().apsSizeX); }
  int16_t getAPSSizeY() const { return (getDeviceInfo().apsSizeY); }

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

  void setStatisticsInterval(double sec) { statsInterval_ = sec; }
  void deviceDisconnected();
  void initializeParameters(CallbackHandler * h);

  void setCoarseFineBias(std::shared_ptr<CoarseFineParameter> p);
  void setVDACBias(std::shared_ptr<VDACParameter> p);
  void setShiftedSourceBias(std::shared_ptr<ShiftedSourceParameter> p);
  void setIntegerParameter(std::shared_ptr<IntegerParameter> p);
  void setBooleanParameter(std::shared_ptr<BooleanParameter> p);

private:
  const DeviceInfo & getDeviceInfo() const { return (device_->getDeviceInfo()); }
  void processingThread();
  void statsThread();
  void printStatistics();
  void startProcessingThread();
  void stopProcessingThread();
  void stopStatsThread();
  void processPacket(uint64_t nsSinceEpoch, const libcaer::events::EventPacket & packet);
  // ------------ variables
  std::shared_ptr<Device> device_;
  CallbackHandler * callbackHandler_{nullptr};
  bool deviceRunning_{false};  // status of device
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
