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

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "robot_server_comm/server/cdr_reader.hpp"
#include "robot_server_comm/server/payload_summary.hpp"

namespace robot_server_comm
{

namespace
{

// Reference messages serialized to CDR by an independent implementation (the
// "rosbags" Python library), so the decoder is checked against real ROS 2 bytes
// and not against an encoder written with the same assumptions.
//
// Every message has stamp 1556262001.123456789. Values:
//   Imu            frame imu_link, gyro (0.01, -0.02, 0.03), accel (0.12, -0.34, 9.81)
//   NavSatFix      frame gps, lat 22.3040123, lon 114.1790456, alt 12.34, status 0, service 1
//   PointCloud2    frame velodyne, 4 x 1 points, fields x y z intensity ring, point_step 18
//   Image          frame camera, 4 x 2, bgr8, step 12, 24 data bytes
//   CompressedImage frame camera, jpeg, 10 data bytes
//   Odometry       frame map, child base_link, position (830123.456, 2470123.789, 12.5),
//                  linear velocity (3, 4, 0), angular velocity (0, 0, 0.1)
//   TwistWithCovarianceStamped frame gps, linear (3, -4, 0.5), angular (0, 0, 0)

const char kImuLittleEndian[] =
  "0001000071acc25c15cd5b0709000000696d755f6c696e6b000000009a9999999999b93f9a9999999999c93f"
  "333333333333d33fcdccccccccccec3f0000000000000000000000000000f03f000000000000004000000000"
  "000008400000000000001040000000000000144000000000000018400000000000001c400000000000002040"
  "7b14ae47e17a843f7b14ae47e17a94bfb81e85eb51b89e3f0000000000000000000000000000f03f00000000"
  "0000004000000000000008400000000000001040000000000000144000000000000018400000000000001c40"
  "0000000000002040b81e85eb51b8be3fc3f5285c8fc2d5bf1f85eb51b89e2340000000000000000000000000"
  "0000f03f00000000000000400000000000000840000000000000104000000000000014400000000000001840"
  "0000000000001c400000000000002040";

const char kImuBigEndian[] =
  "000000005cc2ac71075bcd1500000009696d755f6c696e6b000000003fb999999999999a3fc999999999999a"
  "3fd33333333333333feccccccccccccd00000000000000003ff0000000000000400000000000000040080000"
  "00000000401000000000000040140000000000004018000000000000401c0000000000004020000000000000"
  "3f847ae147ae147bbf947ae147ae147b3f9eb851eb851eb800000000000000003ff000000000000040000000"
  "000000004008000000000000401000000000000040140000000000004018000000000000401c000000000000"
  "40200000000000003fbeb851eb851eb8bfd5c28f5c28f5c340239eb851eb851f00000000000000003ff00000"
  "0000000040000000000000004008000000000000401000000000000040140000000000004018000000000000"
  "401c0000000000004020000000000000";

const char kNavSatFixLittleEndian[] =
  "0001000071acc25c15cd5b0704000000677073000000010000000000ed1406c0d34d3640881fad7b758b5c40"
  "ae47e17a14ae28400000000000000000000000000000f03f0000000000000040000000000000084000000000"
  "00001040000000000000144000000000000018400000000000001c40000000000000204002";

const char kNavSatFixBigEndian[] =
  "000000005cc2ac71075bcd150000000467707300000000010000000040364dd3c00614ed405c8b757bad1f88"
  "4028ae147ae147ae00000000000000003ff00000000000004000000000000000400800000000000040100000"
  "0000000040140000000000004018000000000000401c000000000000402000000000000002";

const char kPointCloud2[] =
  "0001000071acc25c15cd5b070900000076656c6f64796e650000000001000000040000000500000002000000"
  "780000000000000007000000010000000200000079000000040000000700000001000000020000007a000000"
  "0800000007000000010000000a000000696e74656e736974790000000c000000070000000100000005000000"
  "72696e6700000000100000000400000001000000000000001200000048000000480000000001020304050607"
  "08090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f30313233"
  "3435363738393a3b3c3d3e3f404142434445464701";

const char kImage[] =
  "0001000071acc25c15cd5b070700000063616d65726100000200000004000000050000006267723800000000"
  "0c00000018000000000102030405060708090a0b0c0d0e0f1011121314151617";

const char kCompressedImage[] =
  "0001000071acc25c15cd5b070700000063616d6572610000050000006a706567000000000a00000000010203"
  "040506070809";

const char kOdometryLittleEndian[] =
  "0001000071acc25c15cd5b07040000006d6170000a000000626173655f6c696e6b000000fed478e956552941"
  "b6f3fde475d84241000000000000294000000000000000000000000000000000aa36ee83e27dd83f103cbebd"
  "6b90ed3f0000000000000000000000000000f03f000000000000004000000000000008400000000000001040"
  "000000000000144000000000000018400000000000001c400000000000002040000000000000224000000000"
  "00002440000000000000264000000000000028400000000000002a400000000000002c400000000000002e40"
  "0000000000003040000000000000314000000000000032400000000000003340000000000000344000000000"
  "0000354000000000000036400000000000003740000000000000384000000000000039400000000000003a40"
  "0000000000003b400000000000003c400000000000003d400000000000003e400000000000003f4000000000"
  "0000404000000000008040400000000000004140000000000080414000000000000008400000000000001040"
  "0000000000000000000000000000000000000000000000009a9999999999b93f000000000000000000000000"
  "0000f03f00000000000000400000000000000840000000000000104000000000000014400000000000001840"
  "0000000000001c40000000000000204000000000000022400000000000002440000000000000264000000000"
  "000028400000000000002a400000000000002c400000000000002e4000000000000030400000000000003140"
  "0000000000003240000000000000334000000000000034400000000000003540000000000000364000000000"
  "00003740000000000000384000000000000039400000000000003a400000000000003b400000000000003c40"
  "0000000000003d400000000000003e400000000000003f400000000000004040000000000080404000000000"
  "000041400000000000804140";

const char kOdometryBigEndian[] =
  "000000005cc2ac71075bcd15000000046d6170000000000a626173655f6c696e6b00000041295556e978d4fe"
  "4142d875e4fdf3b64029000000000000000000000000000000000000000000003fd87de283ee36aa3fed906b"
  "bdbe3c1000000000000000003ff0000000000000400000000000000040080000000000004010000000000000"
  "40140000000000004018000000000000401c0000000000004020000000000000402200000000000040240000"
  "0000000040260000000000004028000000000000402a000000000000402c000000000000402e000000000000"
  "4030000000000000403100000000000040320000000000004033000000000000403400000000000040350000"
  "000000004036000000000000403700000000000040380000000000004039000000000000403a000000000000"
  "403b000000000000403c000000000000403d000000000000403e000000000000403f00000000000040400000"
  "0000000040408000000000004041000000000000404180000000000040080000000000004010000000000000"
  "0000000000000000000000000000000000000000000000003fb999999999999a00000000000000003ff00000"
  "0000000040000000000000004008000000000000401000000000000040140000000000004018000000000000"
  "401c000000000000402000000000000040220000000000004024000000000000402600000000000040280000"
  "00000000402a000000000000402c000000000000402e00000000000040300000000000004031000000000000"
  "4032000000000000403300000000000040340000000000004035000000000000403600000000000040370000"
  "0000000040380000000000004039000000000000403a000000000000403b000000000000403c000000000000"
  "403d000000000000403e000000000000403f0000000000004040000000000000404080000000000040410000"
  "000000004041800000000000";

const char kTwistWithCovarianceStamped[] =
  "0001000071acc25c15cd5b070400000067707300000000000000084000000000000010c0000000000000e03f"
  "0000000000000000000000000000000000000000000000000000000000000000000000000000f03f00000000"
  "0000004000000000000008400000000000001040000000000000144000000000000018400000000000001c40"
  "0000000000002040000000000000224000000000000024400000000000002640000000000000284000000000"
  "00002a400000000000002c400000000000002e40000000000000304000000000000031400000000000003240"
  "0000000000003340000000000000344000000000000035400000000000003640000000000000374000000000"
  "0000384000000000000039400000000000003a400000000000003b400000000000003c400000000000003d40"
  "0000000000003e400000000000003f4000000000000040400000000000804040000000000000414000000000"
  "00804140";

std::vector<std::uint8_t> from_hex(const std::string & hex)
{
  std::vector<std::uint8_t> bytes;
  bytes.reserve(hex.size() / 2);
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return bytes;
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

}  // namespace

TEST(CdrReader, ReadsPrimitivesStringsAndAlignment)
{
  // Header of the reference IMU message: int32, uint32, string, then doubles aligned on 8.
  const std::vector<std::uint8_t> bytes = from_hex(kImuLittleEndian);
  CdrReader reader(bytes.data(), bytes.size());
  EXPECT_EQ(1556262001, reader.read<std::int32_t>());
  EXPECT_EQ(123456789u, reader.read<std::uint32_t>());
  EXPECT_EQ("imu_link", reader.read_string());
  EXPECT_DOUBLE_EQ(0.1, reader.read<double>());
  EXPECT_DOUBLE_EQ(0.2, reader.read<double>());
  EXPECT_DOUBLE_EQ(0.3, reader.read<double>());
  EXPECT_DOUBLE_EQ(0.9, reader.read<double>());
}

TEST(CdrReader, ReadsBigEndianBuffers)
{
  const std::vector<std::uint8_t> bytes = from_hex(kImuBigEndian);
  CdrReader reader(bytes.data(), bytes.size());
  EXPECT_EQ(1556262001, reader.read<std::int32_t>());
  EXPECT_EQ(123456789u, reader.read<std::uint32_t>());
  EXPECT_EQ("imu_link", reader.read_string());
  EXPECT_DOUBLE_EQ(0.1, reader.read<double>());
}

TEST(CdrReader, ThrowsInsteadOfReadingPastTheEnd)
{
  const std::vector<std::uint8_t> bytes = from_hex(kImuLittleEndian);
  EXPECT_THROW(CdrReader(bytes.data(), 3), CdrError);
  EXPECT_THROW(CdrReader(nullptr, 0), CdrError);

  CdrReader reader(bytes.data(), 10);  // encapsulation + 6 bytes
  EXPECT_EQ(1556262001, reader.read<std::int32_t>());
  EXPECT_THROW(reader.read<std::uint32_t>(), CdrError);

  // A string whose announced length is larger than the buffer.
  const std::vector<std::uint8_t> lying = from_hex("00010000ffffff7f41424344");
  CdrReader lying_reader(lying.data(), lying.size());
  EXPECT_THROW(lying_reader.read_string(), CdrError);

  const std::vector<std::uint8_t> unknown = from_hex("0a0b000000000000");
  EXPECT_THROW(CdrReader(unknown.data(), unknown.size()), CdrError);
}

TEST(PayloadSummary, DescribesAnImuMessage)
{
  const PayloadSummarizer summarizer;
  const std::string text = summarizer.summarize("sensor_msgs/msg/Imu", from_hex(kImuLittleEndian));
  EXPECT_TRUE(contains(text, "stamp=1556262001.123456789")) << text;
  EXPECT_TRUE(contains(text, "frame=imu_link")) << text;
  EXPECT_TRUE(contains(text, "accel=[0.120, -0.340, 9.810] m/s^2")) << text;
  EXPECT_TRUE(contains(text, "gyro=[0.010, -0.020, 0.030] rad/s")) << text;
}

TEST(PayloadSummary, GivesTheSameResultForBothByteOrders)
{
  const PayloadSummarizer summarizer;
  EXPECT_EQ(
    summarizer.summarize("sensor_msgs/msg/Imu", from_hex(kImuLittleEndian)),
    summarizer.summarize("sensor_msgs/msg/Imu", from_hex(kImuBigEndian)));
  EXPECT_EQ(
    summarizer.summarize("sensor_msgs/msg/NavSatFix", from_hex(kNavSatFixLittleEndian)),
    summarizer.summarize("sensor_msgs/msg/NavSatFix", from_hex(kNavSatFixBigEndian)));
  EXPECT_EQ(
    summarizer.summarize("nav_msgs/msg/Odometry", from_hex(kOdometryLittleEndian)),
    summarizer.summarize("nav_msgs/msg/Odometry", from_hex(kOdometryBigEndian)));
}

TEST(PayloadSummary, DescribesAGnssFix)
{
  const PayloadSummarizer summarizer;
  const std::string text =
    summarizer.summarize("sensor_msgs/msg/NavSatFix", from_hex(kNavSatFixLittleEndian));
  EXPECT_TRUE(contains(text, "frame=gps")) << text;
  EXPECT_TRUE(contains(text, "lat=22.3040123")) << text;
  EXPECT_TRUE(contains(text, "lon=114.1790456")) << text;
  EXPECT_TRUE(contains(text, "alt=12.34 m")) << text;
  EXPECT_TRUE(contains(text, "fix_status=0 service=1")) << text;
}

TEST(PayloadSummary, DescribesAPointCloud)
{
  const PayloadSummarizer summarizer;
  const std::string text =
    summarizer.summarize("sensor_msgs/msg/PointCloud2", from_hex(kPointCloud2));
  EXPECT_TRUE(contains(text, "frame=velodyne")) << text;
  EXPECT_TRUE(contains(text, "points=4 (4x1)")) << text;
  EXPECT_TRUE(contains(text, "point_step=18 B")) << text;
  EXPECT_TRUE(contains(text, "fields=x,y,z,intensity,ring")) << text;
  EXPECT_TRUE(contains(text, "data=72 B")) << text;
}

TEST(PayloadSummary, DescribesImages)
{
  const PayloadSummarizer summarizer;
  const std::string raw = summarizer.summarize("sensor_msgs/msg/Image", from_hex(kImage));
  EXPECT_TRUE(contains(raw, "frame=camera")) << raw;
  EXPECT_TRUE(contains(raw, "image=4x2 encoding=bgr8 step=12 data=24 B")) << raw;

  const std::string compressed =
    summarizer.summarize("sensor_msgs/msg/CompressedImage", from_hex(kCompressedImage));
  EXPECT_TRUE(contains(compressed, "format=jpeg data=10 B")) << compressed;
}

TEST(PayloadSummary, DescribesOdometry)
{
  const PayloadSummarizer summarizer;
  const std::string text =
    summarizer.summarize("nav_msgs/msg/Odometry", from_hex(kOdometryLittleEndian));
  EXPECT_TRUE(contains(text, "stamp=1556262001.123456789 frame=map child=base_link")) << text;
  EXPECT_TRUE(contains(text, "position=[830123.456, 2470123.789, 12.500] m")) << text;
  EXPECT_TRUE(contains(text, "velocity=[3.000, 4.000, 0.000] m/s speed=5.00 m/s")) << text;
}

TEST(PayloadSummary, DescribesAGnssVelocity)
{
  const PayloadSummarizer summarizer;
  const std::string text = summarizer.summarize(
    "geometry_msgs/msg/TwistWithCovarianceStamped", from_hex(kTwistWithCovarianceStamped));
  EXPECT_TRUE(contains(text, "frame=gps")) << text;
  EXPECT_TRUE(contains(text, "linear=[3.000, -4.000, 0.500] m/s speed=5.02 m/s")) << text;
  EXPECT_TRUE(contains(text, "angular=[0.000, 0.000, 0.000] rad/s")) << text;
}

TEST(PayloadSummary, FallsBackToAHexPreviewForUnknownTypes)
{
  const PayloadSummarizer summarizer;
  EXPECT_FALSE(summarizer.knows("novatel_msgs/msg/INSPVAX"));
  const std::string text =
    summarizer.summarize("novatel_msgs/msg/INSPVAX", from_hex("000100000102030405"));
  EXPECT_EQ("raw bytes: 00 01 00 00 01 02 03 04 05", text);
  EXPECT_EQ("raw bytes:", summarizer.summarize("unknown/msg/Type", {}));
}

TEST(PayloadSummary, NeverThrowsOnTruncatedOrCorruptMessages)
{
  const PayloadSummarizer summarizer;
  const std::vector<std::uint8_t> imu = from_hex(kImuLittleEndian);
  // Every possible truncation of a valid message must be handled. The summary
  // needs the message up to the linear acceleration, which ends at byte 252.
  constexpr std::size_t kBytesNeeded = 252;
  for (std::size_t size = 0; size < imu.size(); ++size) {
    const std::vector<std::uint8_t> truncated(imu.begin(), imu.begin() + size);
    const std::string text = summarizer.summarize("sensor_msgs/msg/Imu", truncated);
    EXPECT_EQ(size < kBytesNeeded, contains(text, "could not decode")) <<
      "size " << size << ": " << text;
  }
  const std::vector<std::uint8_t> cloud = from_hex(kPointCloud2);
  for (std::size_t size = 0; size < cloud.size(); size += 7) {
    const std::vector<std::uint8_t> truncated(cloud.begin(), cloud.begin() + size);
    EXPECT_NO_THROW(summarizer.summarize("sensor_msgs/msg/PointCloud2", truncated));
  }
  const std::vector<std::uint8_t> odometry = from_hex(kOdometryLittleEndian);
  for (std::size_t size = 0; size < odometry.size(); ++size) {
    const std::vector<std::uint8_t> truncated(odometry.begin(), odometry.begin() + size);
    EXPECT_NO_THROW(summarizer.summarize("nav_msgs/msg/Odometry", truncated));
  }
  const std::vector<std::uint8_t> velocity = from_hex(kTwistWithCovarianceStamped);
  for (std::size_t size = 0; size < velocity.size(); ++size) {
    const std::vector<std::uint8_t> truncated(velocity.begin(), velocity.begin() + size);
    EXPECT_NO_THROW(
      summarizer.summarize("geometry_msgs/msg/TwistWithCovarianceStamped", truncated));
  }
  // A point cloud that claims an absurd number of fields.
  std::vector<std::uint8_t> absurd = cloud;
  // Bytes 36 to 39 hold the number of fields (after header, height and width).
  absurd[36] = 0xFF;
  absurd[37] = 0xFF;
  absurd[38] = 0xFF;
  absurd[39] = 0x7F;
  EXPECT_TRUE(
    contains(summarizer.summarize("sensor_msgs/msg/PointCloud2", absurd), "could not decode"));
}

TEST(PayloadSummary, AcceptsDecodersForNewMessageTypes)
{
  PayloadSummarizer summarizer;
  summarizer.register_decoder(
    "my_msgs/msg/Battery", [](const std::uint8_t *, std::size_t size) {
      return "battery message of " + std::to_string(size) + " bytes";
    });
  EXPECT_TRUE(summarizer.knows("my_msgs/msg/Battery"));
  EXPECT_EQ(
    "battery message of 3 bytes", summarizer.summarize("my_msgs/msg/Battery", {1, 2, 3}));
}

TEST(PayloadSummary, FormatsSizes)
{
  EXPECT_EQ("0 B", format_size(0));
  EXPECT_EQ("1023 B", format_size(1023));
  EXPECT_EQ("1.0 kB", format_size(1024));
  EXPECT_EQ("1.50 MB", format_size(1572864));
}

}  // namespace robot_server_comm
