// Copyright 2026 Anas Derkaoui
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

#include "robot_server_comm/server/payload_summary.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "robot_server_comm/server/cdr_reader.hpp"

namespace robot_server_comm
{

namespace
{

// Point clouds describe their layout with a list of fields. A sane message has a handful.
constexpr std::uint32_t kMaxPointFields = 256;

std::string format_number(const char * format, double value)
{
  char text[64];
  std::snprintf(text, sizeof(text), format, value);
  return text;
}

std::string format_vector3(double x, double y, double z)
{
  return "[" + format_number("%.3f", x) + ", " + format_number("%.3f", y) + ", " +
         format_number("%.3f", z) + "]";
}

// std_msgs/msg/Header: time stamp (int32 sec, uint32 nanosec) and frame id.
struct Header
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};
  std::string frame_id;

  std::string to_string() const
  {
    char stamp[48];
    std::snprintf(stamp, sizeof(stamp), "%" PRId32 ".%09" PRIu32, sec, nanosec);
    return "stamp=" + std::string(stamp) + " frame=" + (frame_id.empty() ? "''" : frame_id);
  }
};

Header read_header(CdrReader & reader)
{
  Header header;
  header.sec = reader.read<std::int32_t>();
  header.nanosec = reader.read<std::uint32_t>();
  header.frame_id = reader.read_string();
  return header;
}

std::string summarize_imu(const std::uint8_t * data, std::size_t size)
{
  CdrReader reader(data, size);
  const Header header = read_header(reader);
  reader.skip<double>(4);  // orientation quaternion
  reader.skip<double>(9);  // orientation covariance
  const double gyro_x = reader.read<double>();
  const double gyro_y = reader.read<double>();
  const double gyro_z = reader.read<double>();
  reader.skip<double>(9);  // angular velocity covariance
  const double accel_x = reader.read<double>();
  const double accel_y = reader.read<double>();
  const double accel_z = reader.read<double>();
  return header.to_string() + " accel=" + format_vector3(accel_x, accel_y, accel_z) +
         " m/s^2 gyro=" + format_vector3(gyro_x, gyro_y, gyro_z) + " rad/s";
}

std::string summarize_nav_sat_fix(const std::uint8_t * data, std::size_t size)
{
  CdrReader reader(data, size);
  const Header header = read_header(reader);
  const int status = reader.read<std::int8_t>();
  const unsigned service = reader.read<std::uint16_t>();
  const double latitude = reader.read<double>();
  const double longitude = reader.read<double>();
  const double altitude = reader.read<double>();
  return header.to_string() + " lat=" + format_number("%.7f", latitude) +
         " lon=" + format_number("%.7f", longitude) +
         " alt=" + format_number("%.2f", altitude) + " m fix_status=" + std::to_string(status) +
         " service=" + std::to_string(service);
}

std::string format_speed(double x, double y, double z)
{
  return format_number("%.2f", std::sqrt(x * x + y * y + z * z)) + " m/s";
}

// nav_msgs/msg/Odometry: header, child frame, pose with covariance, twist with covariance.
std::string summarize_odometry(const std::uint8_t * data, std::size_t size)
{
  CdrReader reader(data, size);
  const Header header = read_header(reader);
  const std::string child_frame = reader.read_string();
  const double x = reader.read<double>();
  const double y = reader.read<double>();
  const double z = reader.read<double>();
  reader.skip<double>(4);   // orientation quaternion
  reader.skip<double>(36);  // pose covariance
  const double vx = reader.read<double>();
  const double vy = reader.read<double>();
  const double vz = reader.read<double>();
  return header.to_string() + " child=" + (child_frame.empty() ? "''" : child_frame) +
         " position=" + format_vector3(x, y, z) + " m velocity=" + format_vector3(vx, vy, vz) +
         " m/s speed=" + format_speed(vx, vy, vz);
}

// geometry_msgs/msg/TwistWithCovarianceStamped: header, linear and angular velocity, covariance.
std::string summarize_twist_with_covariance_stamped(const std::uint8_t * data, std::size_t size)
{
  CdrReader reader(data, size);
  const Header header = read_header(reader);
  const double vx = reader.read<double>();
  const double vy = reader.read<double>();
  const double vz = reader.read<double>();
  const double wx = reader.read<double>();
  const double wy = reader.read<double>();
  const double wz = reader.read<double>();
  return header.to_string() + " linear=" + format_vector3(vx, vy, vz) + " m/s speed=" +
         format_speed(vx, vy, vz) + " angular=" + format_vector3(wx, wy, wz) + " rad/s";
}

std::string summarize_point_cloud2(const std::uint8_t * data, std::size_t size)
{
  CdrReader reader(data, size);
  const Header header = read_header(reader);
  const std::uint32_t height = reader.read<std::uint32_t>();
  const std::uint32_t width = reader.read<std::uint32_t>();

  const std::uint32_t field_count = reader.read<std::uint32_t>();
  if (field_count > kMaxPointFields) {
    throw CdrError("implausible number of point fields");
  }
  std::string fields;
  for (std::uint32_t i = 0; i < field_count; ++i) {
    const std::string name = reader.read_string();
    reader.read<std::uint32_t>();  // offset
    reader.read<std::uint8_t>();   // datatype
    reader.read<std::uint32_t>();  // count
    fields += (i == 0 ? "" : ",") + name;
  }
  reader.read<std::uint8_t>();  // is_bigendian
  const std::uint32_t point_step = reader.read<std::uint32_t>();
  reader.read<std::uint32_t>();  // row_step
  const std::uint32_t data_length = reader.read<std::uint32_t>();

  const std::uint64_t points = static_cast<std::uint64_t>(height) * width;
  return header.to_string() + " points=" + std::to_string(points) +
         " (" + std::to_string(width) + "x" + std::to_string(height) + ")" +
         " point_step=" + std::to_string(point_step) + " B fields=" + fields +
         " data=" + format_size(data_length);
}

std::string summarize_image(const std::uint8_t * data, std::size_t size)
{
  CdrReader reader(data, size);
  const Header header = read_header(reader);
  const std::uint32_t height = reader.read<std::uint32_t>();
  const std::uint32_t width = reader.read<std::uint32_t>();
  const std::string encoding = reader.read_string();
  reader.read<std::uint8_t>();  // is_bigendian
  const std::uint32_t step = reader.read<std::uint32_t>();
  const std::uint32_t data_length = reader.read<std::uint32_t>();
  return header.to_string() + " image=" + std::to_string(width) + "x" + std::to_string(height) +
         " encoding=" + encoding + " step=" + std::to_string(step) +
         " data=" + format_size(data_length);
}

std::string summarize_compressed_image(const std::uint8_t * data, std::size_t size)
{
  CdrReader reader(data, size);
  const Header header = read_header(reader);
  const std::string format = reader.read_string();
  const std::uint32_t data_length = reader.read<std::uint32_t>();
  return header.to_string() + " format=" + format + " data=" + format_size(data_length);
}

std::string hex_preview(const std::vector<std::uint8_t> & payload)
{
  constexpr std::size_t kPreviewBytes = 16;
  std::string text = "raw bytes:";
  const std::size_t count = std::min(payload.size(), kPreviewBytes);
  for (std::size_t i = 0; i < count; ++i) {
    char byte[4];
    std::snprintf(byte, sizeof(byte), " %02x", static_cast<unsigned>(payload[i]));
    text += byte;
  }
  if (payload.size() > count) {
    text += " ...";
  }
  return text;
}

}  // namespace

std::string format_size(std::size_t bytes)
{
  const double value = static_cast<double>(bytes);
  if (bytes < 1024u) {
    return std::to_string(bytes) + " B";
  }
  if (bytes < 1024u * 1024u) {
    return format_number("%.1f", value / 1024.0) + " kB";
  }
  return format_number("%.2f", value / (1024.0 * 1024.0)) + " MB";
}

PayloadSummarizer::PayloadSummarizer()
{
  register_decoder("sensor_msgs/msg/Imu", summarize_imu);
  register_decoder("sensor_msgs/msg/NavSatFix", summarize_nav_sat_fix);
  register_decoder("sensor_msgs/msg/PointCloud2", summarize_point_cloud2);
  register_decoder("sensor_msgs/msg/Image", summarize_image);
  register_decoder("sensor_msgs/msg/CompressedImage", summarize_compressed_image);
  register_decoder("nav_msgs/msg/Odometry", summarize_odometry);
  register_decoder(
    "geometry_msgs/msg/TwistWithCovarianceStamped", summarize_twist_with_covariance_stamped);
}

void PayloadSummarizer::register_decoder(const std::string & message_type, Decoder decoder)
{
  decoders_[message_type] = std::move(decoder);
}

bool PayloadSummarizer::knows(const std::string & message_type) const
{
  return decoders_.count(message_type) > 0;
}

std::string PayloadSummarizer::summarize(
  const std::string & message_type, const std::vector<std::uint8_t> & payload) const
{
  const auto found = decoders_.find(message_type);
  if (found == decoders_.end()) {
    return hex_preview(payload);
  }
  try {
    return found->second(payload.data(), payload.size());
  } catch (const std::exception & error) {
    return std::string("could not decode (") + error.what() + "), " + hex_preview(payload);
  }
}

}  // namespace robot_server_comm
