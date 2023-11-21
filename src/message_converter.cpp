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

#include <libcaer_driver/message_converter.hpp>
#include <libcaer_driver/resize_hack.hpp>
#include <rclcpp/rclcpp.hpp>

namespace libcaer_driver
{
namespace message_converter
{
// local logger handle
static rclcpp::Logger logger() { return (rclcpp::get_logger("driver")); }

size_t convert_polarity_packet(
  event_camera_msgs::msg::EventPacket * msg, const libcaer::events::PolarityEventPacket & packet,
  const rclcpp::Time & baseTime)
{
  if (msg->events.empty() && (packet.getEventNumber() > 0)) {
    msg->time_base = packet[0].getTimestamp64(packet) * 1000;
    msg->header.stamp = baseTime + rclcpp::Duration::from_nanoseconds(msg->time_base);
  }

  const size_t BYTES_PER_ENCODED_EVENT = 8;
  const size_t n = packet.getEventNumber() * BYTES_PER_ENCODED_EVENT;
  const size_t oldSize = msg->events.size();
  libcaer_driver::resize_hack(msg->events, oldSize + n);

  uint64_t * p = reinterpret_cast<uint64_t *>(msg->events.data() + oldSize);
  for (int32_t i = 0; i < packet.getEventNumber(); i++, p++) {
    const auto & e = packet[i];
    const auto t = e.getTimestamp64(packet) * 1000;
    const uint64_t dt = t - msg->time_base;
    *p = static_cast<uint64_t>(e.getPolarity()) << 63 | static_cast<uint64_t>(e.getY()) << 48 |
         static_cast<uint64_t>(e.getX()) << 32 | (dt & 0xFFFFFFFFULL);
  }
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
      RCLCPP_ERROR_STREAM(logger(), "invalid number of channels for frame: " << numChan);
      throw(std::runtime_error("invalid number of channels for frame"));
  }
  msg->header.stamp =
    baseTime + rclcpp::Duration::from_nanoseconds(frame.getTimestamp64(packet) * 1000);
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
    baseTime + rclcpp::Duration::from_nanoseconds(imu.getTimestamp64(packet) * 1000);
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
