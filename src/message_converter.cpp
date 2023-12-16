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

#include <bitset>
#include <libcaer_driver/compression_tokens.hpp>
#include <libcaer_driver/logging.hpp>
#include <libcaer_driver/message_converter.hpp>
#include <libcaer_driver/resize_hack.hpp>
#include <rclcpp/rclcpp.hpp>

namespace libcaer_driver
{
namespace message_converter
{
// local logger handle
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("driver")); }

//
// ---------- uncompressed libcaer encoding
//

size_t convert_polarity_packet(
  event_camera_msgs::msg::EventPacket * msg, const libcaer::events::PolarityEventPacket & packet,
  const rclcpp::Time & baseTime)
{
  if (msg->events.empty() && (packet.getEventNumber() > 0)) {
    const uint64_t sensorTime = packet[0].getTimestamp64(packet);
    // remove the lower bits from the sensor time to just keep the "overflow" bits
    const uint64_t sensorTimeHighBits = sensorTime & (~((1ULL << TS_OVERFLOW_SHIFT) - 1));
    // the time_base has the ros base time (including the lower bits!), increased
    // by the high bits of the elapsed sensor time
    msg->time_base = sensorTimeHighBits * 1000 + baseTime.nanoseconds();
  }

  const size_t BYTES_PER_ENCODED_EVENT = 8;
  const size_t n = packet.getEventNumber() * BYTES_PER_ENCODED_EVENT;
  const size_t oldSize = msg->events.size();
  libcaer_driver::resize_hack(msg->events, oldSize + n);
  memcpy(
    reinterpret_cast<void *>(msg->events.data() + oldSize),
    reinterpret_cast<const void *>(&packet[0]), n);
  return (packet.getEventNumber());
}

//
// ---------- compressed libcaer_cmp encoding
//

template <class T>
static void add_to_events(std::vector<uint8_t> * events, const T & token)
{
  const uint8_t * p = reinterpret_cast<const uint8_t *>(&token);
  events->push_back(p[0]);  // token must be 16 bytes or else!
  events->push_back(p[1]);
}

static inline void flush_state(
  std::vector<uint8_t> * events, std::bitset<8> * currentMask, uint16_t currentY,
  uint16_t currentYHigh, uint8_t currentPolarity)
{
  if (currentMask->any()) {
    if (currentMask->count() == 1) {
      add_to_events<AddrY>(events, AddrY(currentY, currentPolarity));
    } else {
      add_to_events(events, VectBaseY(currentYHigh, currentPolarity));
      add_to_events(events, Vect8(static_cast<uint16_t>(currentMask->to_ulong())));
    }
    currentMask->reset();
  }
}

size_t convert_polarity_packet_compressed(
  event_camera_msgs::msg::EventPacket * msg, const libcaer::events::PolarityEventPacket & packet,
  const rclcpp::Time & baseTime, uint64_t * sensorTime_0)
{
  if (packet.getEventNumber() == 0) {
    return (packet.getEventNumber());
  }
  constexpr uint64_t TIME_MASK = (1ULL << 24) - 1ULL;
  constexpr uint64_t TIME_LOW_MASK = (1ULL << 12) - 1ULL;
  constexpr uint64_t TIME_HIGH_MASK = TIME_MASK & (~TIME_LOW_MASK);
  constexpr uint16_t Y_LOW_MASK = 0x0007;        // lowest 3 bits
  constexpr uint16_t Y_HIGH_MASK = ~Y_LOW_MASK;  // highest 13 bits
  const uint64_t sensorPacketTime = packet[0].getTimestamp64(packet);
  auto & events = msg->events;
  if (events.empty()) {
    // base time is simply the ROS time at startup + sensor elapsed time
    msg->time_base = sensorPacketTime * 1000 + baseTime.nanoseconds();
    *sensorTime_0 = sensorPacketTime;
  }
  const uint64_t dt_0 = sensorPacketTime - *sensorTime_0;
  uint16_t timeHigh = dt_0 & TIME_HIGH_MASK;
  uint64_t timeLow = dt_0 & TIME_LOW_MASK;

  uint16_t currentX = packet[0].getX();
  add_to_events(&events, AddrX(currentX));

  uint16_t currentY = std::numeric_limits<uint16_t>::max();
  uint16_t currentYHigh = currentY & Y_HIGH_MASK;
  std::bitset<8> currentMask(0);
  uint8_t currentPolarity = 2;

  for (int i = 0; i < packet.getEventNumber(); i++) {
    const auto & p = packet[i];
    const uint64_t dt = p.getTimestamp64(packet) - *sensorTime_0;
    if ((dt & TIME_HIGH_MASK) != timeHigh) {
      timeHigh = dt & TIME_HIGH_MASK;
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      add_to_events(&events, TimeHigh(static_cast<uint16_t>(timeHigh >> 12)));
    }
    if ((dt & TIME_LOW_MASK) != timeLow) {
      timeLow = dt & TIME_LOW_MASK;
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      add_to_events(&events, TimeLow(static_cast<uint16_t>(timeLow)));
    }
    if (p.getX() != currentX) {
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      currentX = p.getX();
      add_to_events(&events, AddrX(currentX));
    }
    if (p.getPolarity() != currentPolarity) {
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      currentPolarity = p.getPolarity();
    }

    const uint16_t y = p.getY();
    const uint16_t y_high = y & Y_HIGH_MASK;
    const uint16_t y_low = y & Y_LOW_MASK;
    if (y_high != currentYHigh) {
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      currentYHigh = y_high;
      currentY = y;  // for easier access in case of sparse event
    }
    // mark the current bit in the mask
    currentMask.set(y_low, 1);
  }

  flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
  return (packet.getEventNumber());
}

static std::unique_ptr<sensor_msgs::msg::Image> convert_frame(
  const libcaer::events::FrameEvent & frame, const libcaer::events::FrameEventPacket & packet,
  const std::string & frameId, const rclcpp::Time & baseTime)
{
  std::unique_ptr<sensor_msgs::msg::Image> msg(new sensor_msgs::msg::Image());
  msg->height = frame.getLengthY();
  msg->width = frame.getLengthX();
  const uint16_t * data = frame.getPixelArrayUnsafe();
  uint32_t numChan = static_cast<uint32_t>(frame.getChannelNumber());
  switch (frame.getChannelNumber()) {
    case libcaer::events::FrameEvent::colorChannels::GRAYSCALE:
      msg->encoding = "mono8";
      break;
    case libcaer::events::FrameEvent::colorChannels::RGB:
      msg->encoding = "rgb8";
      break;
    case libcaer::events::FrameEvent::colorChannels::RGBA:
      msg->encoding = "rgba8";
      break;
    default:
      BOMB_OUT("invalid number of channels for frame: " << numChan);
  }
  msg->header.stamp =
    baseTime + rclcpp::Duration(std::chrono::nanoseconds(frame.getTimestamp64(packet) * 1000));
  msg->header.frame_id = frameId;

  const uint32_t stride = numChan * (msg->width);
  msg->step = stride;
  msg->data.resize(msg->step * msg->height);
  for (uint32_t y = 0; y < msg->height * numChan; y++) {
    for (uint32_t x = 0; x < msg->width; x++) {
      // convert from 16bit to 8bit.
      msg->data[y * (msg->width) + x] = data[y * (msg->width) + x] >> 8;
    }
  }
  return (msg);
}

size_t convert_frame_packet(
  std::vector<std::unique_ptr<sensor_msgs::msg::Image>> * msgs,
  const libcaer::events::FrameEventPacket & packet, const std::string & frameId,
  const rclcpp::Time & baseTime)
{
  for (int32_t i = 0; i < packet.getEventNumber(); i++) {
    const auto & frame = packet[i];  // grab first frame in packet
    msgs->push_back(convert_frame(frame, packet, frameId, baseTime));
  }
  return (msgs->size());
}

static std::unique_ptr<sensor_msgs::msg::Imu> convert_imu(
  const libcaer::events::IMU6Event & imu, const libcaer::events::IMU6EventPacket & packet,
  const std::string & frameId, const rclcpp::Time & baseTime)
{
  constexpr double G = 9.81;
  constexpr double DEG_TO_RADIANS = M_PI / 180.0;
  std::unique_ptr<sensor_msgs::msg::Imu> msg(new sensor_msgs::msg::Imu());
  msg->angular_velocity.x = imu.getGyroX() * DEG_TO_RADIANS;
  msg->angular_velocity.y = imu.getGyroY() * DEG_TO_RADIANS;
  msg->angular_velocity.z = imu.getGyroZ() * DEG_TO_RADIANS;
  msg->linear_acceleration.x = imu.getAccelX() * G;
  msg->linear_acceleration.y = imu.getAccelY() * G;
  msg->linear_acceleration.z = imu.getAccelZ() * G;
  msg->orientation_covariance[0] = -1.0;  // see ROS REP 145
  msg->header.frame_id = frameId;
  msg->header.stamp =
    baseTime + rclcpp::Duration(std::chrono::nanoseconds(imu.getTimestamp64(packet) * 1000));
  return (msg);
}

size_t convert_imu6_packet(
  std::vector<std::unique_ptr<sensor_msgs::msg::Imu>> * msgs,
  const libcaer::events::IMU6EventPacket & packet, const std::string & frameId,
  const rclcpp::Time & baseTime)
{
  for (int32_t i = 0; i < packet.getEventNumber(); i++) {
    const auto & imu = packet[i];  // grab first frame in packet
    msgs->push_back(convert_imu(imu, packet, frameId, baseTime));
  }
  return (msgs->size());
}

}  // namespace message_converter
}  // namespace libcaer_driver
