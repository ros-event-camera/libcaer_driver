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

#include <libcaer_driver/callback_handler.hpp>
#include <libcaer_driver/parameter.hpp>
#include <libcaer_driver/coarse_fine_parameter.hpp>
#include <libcaer_driver/vdac_parameter.hpp>
#include <libcaer_driver/integer_parameter.hpp>
#include <libcaer_driver/boolean_parameter.hpp>

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
  struct DevInfo
  {
    // -------- common to Davis and DVXplorer
    int16_t deviceID{-1};
    std::string deviceSerialNumber;
    uint8_t deviceUSBBusNumber;
    uint8_t deviceUSBDeviceAddress;
    std::string deviceString;
    int16_t firmwareVersion{-1};
    int16_t logicVersion{-1};
    int16_t chipID{-1};
    bool deviceIsMaster{true};
    bool muxHasStatistics{false};
    int16_t dvsSizeX{0};
    int16_t dvsSizeY{0};
    bool dvsHasStatistics{false};
    enum caer_imu_types imuType;
    bool extInputHasGenerator{false};
    // -------- only valid for Davis
    bool dvsHasPixelFilter{false};
    bool dvsHasBackgroundActivityFilter;
    bool dvsHasROIFilter{false};
    bool dvsHasSkipFilter{false};
    bool dvsHasPolarityFilter{false};
    int16_t apsSizeX{0};
    int16_t apsSizeY{0};
    enum caer_frame_event_color_filter apsColorFilter;
    bool apsHasGlobalShutter{false};
  };

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
  const DevInfo &getInfo() const { return (devInfo_);}

  void setStatisticsInterval(double sec) { statsInterval_ = sec; }
  void deviceDisconnected();
  void initializeParameters(CallbackHandler *h);
  const Parameters &getParameters();

  Value setCourseFineBias(const std::string &name, std::shared_ptr<CoarseFineParameter> p, int32_t targetBias);
  Value setVDACBias(const std::string &name, std::shared_ptr<VDACParameter> p, int32_t targetBias);
  Value setIntegerParameter(const std::string &name, std::shared_ptr<IntegerParameter> p, int32_t targetValue);
  bool setBooleanParameter(std::shared_ptr<BooleanParameter> p, bool targetValue);

private:
  void processingThread();
  void statsThread();
  void printStatistics();
  void startProcessingThread();
  void stopProcessingThread();
  void stopStatsThread();
  void processPacket(uint64_t nsSinceEpoch, const libcaer::events::EventPacket & packet);
  uint32_t configGet(int8_t modAddr, uint8_t paramAddr);
  // ------------ variables
  CallbackHandler * callbackHandler_{nullptr};
  int16_t deviceType_{0};
  bool isMaster_{true};  // whether it is configured as master or not
  DevInfo devInfo_;      // all device info
  std::unique_ptr<libcaer::devices::device> device_;
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
