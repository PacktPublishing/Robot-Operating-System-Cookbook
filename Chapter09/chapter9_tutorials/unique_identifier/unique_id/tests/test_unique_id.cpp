//
// C++ unit tests for unique_id interface.
//
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <unique_id/unique_id.h>

using namespace unique_id;
typedef boost::uuids::uuid uuid;
typedef uuid_msgs::UniqueID UniqueID;

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test random generator
TEST(BoostUUID, fromRandom)
{
  static const int N = 1000;
  uuid uu[N];
  for (int i = 0; i < N; ++i)
    {
      uu[i] = fromRandom();
      for (int j = i-1; j >= 0; --j)
        {
          EXPECT_NE(uu[i], uu[j]);
        }
    }
}

// Test Timebased UUID
TEST(BoostUUID, fromTime)
{
  ros::Time t(1515778146, 239216089);
  uint64_t hw_addr = 0xAABBCCDDEEFF;

  // Generate UUID from time and hardware address
  uuid uu = fromTime(t, hw_addr);

  // Obtain time since epoch back from the generated uuid
  std::vector<uint8_t> data(uu.size());
  std::copy(uu.begin(), uu.end(), data.begin());

  uint64_t timestamp = (static_cast<uint64_t>(data[0]) << 24) + (static_cast<uint64_t>(data[1]) << 16) +
                       (static_cast<uint64_t>(data[2]) << 8) + static_cast<uint64_t>(data[3]) +
                       (static_cast<uint64_t>(data[4]) << 40) + (static_cast<uint64_t>(data[5]) << 32) +
                       (static_cast<uint64_t>(data[6] & 0x0F) << 56) + (static_cast<uint64_t>(data[7]) << 48);
  uint64_t offset = 122192928000000000;  // time offset in 100 ns intervals between RFC4122 Timestamp and Epoch Time
  uint64_t ns_intervals = timestamp - offset;
  uint64_t uu_hw_addr = 0;
  for (int8_t i=0; i < 6; i++)
  {
    uu_hw_addr += static_cast<uint64_t>(data[uu.size() - 1 - i]) << 8*i;
  }

  ros::Time uu_time;
  uu_time.sec = static_cast<int32_t>(ns_intervals / 1e9 * 100);
  uu_time.nsec = static_cast<int32_t>((ns_intervals - static_cast<uint64_t>(uu_time.sec) / (100 * 1e-9)) * 100);

  EXPECT_EQ(t.sec, uu_time.sec);
  // The UUID stores the timestamp in 100ns intervals,
  // so we lose precision for the 1's and 10's value of the nanoseconds in the time given
  EXPECT_EQ(t.nsec - uu_time.nsec, 89);
  EXPECT_EQ(hw_addr, uu_hw_addr);
}

TEST(BoostUUID, timeWithinSameInterval)
{
  ros::Time t1(1515778146, 239216020);
  ros::Time t2(1515778146, 239216080);
  uint64_t hw_addr = 0xAABBCCDDEEFF;

  // Generate UUIDs from times and hardware address
  uuid uu1 = fromTime(t1, hw_addr);
  uuid uu2 = fromTime(t2, hw_addr);

  // Obtain times since epoch back from uu1
  std::vector<uint8_t> data(uu1.size());
  std::copy(uu1.begin(), uu1.end(), data.begin());

  uint64_t timestamp = (static_cast<uint64_t>(data[0]) << 24) + (static_cast<uint64_t>(data[1]) << 16) +
                       (static_cast<uint64_t>(data[2]) << 8) + static_cast<uint64_t>(data[3]) +
                       (static_cast<uint64_t>(data[4]) << 40) + (static_cast<uint64_t>(data[5]) << 32) +
                       (static_cast<uint64_t>(data[6] & 0x0F) << 56) + (static_cast<uint64_t>(data[7]) << 48);
  uint64_t offset = 122192928000000000;  // time offset in 100 ns intervals between RFC4122 Timestamp and Epoch Time
  uint64_t ns_intervals = timestamp - offset;

  ros::Time uu1_time;
  uu1_time.sec = static_cast<int32_t>(ns_intervals / 1e9 * 100);
  uu1_time.nsec = static_cast<int32_t>((ns_intervals - static_cast<uint64_t>(uu1_time.sec) / (100 * 1e-9)) * 100);

  // Obtain times since epoch back from uu2
  std::copy(uu2.begin(), uu2.end(), data.begin());

  timestamp = (static_cast<uint64_t>(data[0]) << 24) + (static_cast<uint64_t>(data[1]) << 16) +
              (static_cast<uint64_t>(data[2]) << 8) + static_cast<uint64_t>(data[3]) +
              (static_cast<uint64_t>(data[4]) << 40) + (static_cast<uint64_t>(data[5]) << 32) +
              (static_cast<uint64_t>(data[6] & 0x0F) << 56) + (static_cast<uint64_t>(data[7]) << 48);
  ns_intervals = timestamp - offset;

  ros::Time uu2_time;
  uu2_time.sec = static_cast<int32_t>(ns_intervals / 1e9 * 100);
  uu2_time.nsec = static_cast<int32_t>((ns_intervals - static_cast<uint64_t>(uu2_time.sec) / (100 * 1e-9)) * 100);

  // Compare
  // Since both timestamps were in the same 100ns interval, should get same timestamp
  EXPECT_EQ(uu1_time.sec, uu2_time.sec);
  EXPECT_EQ(uu1_time.nsec, uu2_time.nsec);

  // While timestamps are the same, the clock_id is a randomly generated 14-bit value
  // So, we should still expect the uuids generated to be different
  EXPECT_NE(uu1, uu2);
}

TEST(BoostUUID, timeWithinDifferentInterval)
{
  ros::Time t1(1515778146, 239216020);
  ros::Time t2(1515778146, 239216120);
  uint64_t hw_addr = 0xAABBCCDDEEFF;

  // Generate UUIDs from times and hardware address
  uuid uu1 = fromTime(t1, hw_addr);
  uuid uu2 = fromTime(t2, hw_addr);

  // Obtain times since epoch back from uu1
  std::vector<uint8_t> data(uu1.size());
  std::copy(uu1.begin(), uu1.end(), data.begin());

  uint64_t timestamp = (static_cast<uint64_t>(data[0]) << 24) + (static_cast<uint64_t>(data[1]) << 16) +
                       (static_cast<uint64_t>(data[2]) << 8) + static_cast<uint64_t>(data[3]) +
                       (static_cast<uint64_t>(data[4]) << 40) + (static_cast<uint64_t>(data[5]) << 32) +
                       (static_cast<uint64_t>(data[6] & 0x0F) << 56) + (static_cast<uint64_t>(data[7]) << 48);
  uint64_t offset = 122192928000000000;  // time offset in 100 ns intervals between RFC4122 Timestamp and Epoch Time
  uint64_t ns_intervals = timestamp - offset;

  ros::Time uu1_time;
  uu1_time.sec = static_cast<int32_t>(ns_intervals / 1e9 * 100);
  uu1_time.nsec = static_cast<int32_t>((ns_intervals - static_cast<uint64_t>(uu1_time.sec) / (100 * 1e-9)) * 100);

  // Obtain times since epoch back from uu2
  std::copy(uu2.begin(), uu2.end(), data.begin());

  timestamp = (static_cast<uint64_t>(data[0]) << 24) + (static_cast<uint64_t>(data[1]) << 16) +
              (static_cast<uint64_t>(data[2]) << 8) + static_cast<uint64_t>(data[3]) +
              (static_cast<uint64_t>(data[4]) << 40) + (static_cast<uint64_t>(data[5]) << 32) +
              (static_cast<uint64_t>(data[6] & 0x0F) << 56) + (static_cast<uint64_t>(data[7]) << 48);
  ns_intervals = timestamp - offset;

  ros::Time uu2_time;
  uu2_time.sec = static_cast<int32_t>(ns_intervals / 1e9 * 100);
  uu2_time.nsec = static_cast<int32_t>((ns_intervals - static_cast<uint64_t>(uu2_time.sec) / (100 * 1e-9)) * 100);

  // Compare
  // Since both timestamps were in different 100ns intervals, should get different timestamps (at the ns level)
  EXPECT_EQ(uu1_time.sec, uu2_time.sec);
  EXPECT_NE(uu1_time.nsec, uu2_time.nsec);
}

TEST(BoostUUID, emptyURL)
{
  std::string s;
  uuid x = fromURL(s);
  uuid y = fromURL(s);
  EXPECT_EQ(x, y);
  // MUST yield same result as Python fromURL() function:
  EXPECT_EQ(toHexString(x), "1b4db7eb-4057-5ddf-91e0-36dec72071f5");
}

TEST(BoostUUID, sameURL)
{
  std::string s("http://openstreetmap.org/node/1");
  uuid x = fromURL(s);
  uuid y = fromURL(s);
  EXPECT_EQ(x, y);
  // MUST yield same result as Python fromURL() function:
  EXPECT_EQ(toHexString(x), "ef362ac8-9659-5481-b954-88e9b741c8f9");
}

TEST(BoostUUID, differentOsmNamespace)
{
  uuid x = fromURL("http://openstreetmap.org/node/1");
  uuid y = fromURL("http://openstreetmap.org/way/1");
  EXPECT_NE(x, y);
  // MUST yield same result as Python fromURL() function:
  EXPECT_EQ(toHexString(y), "b3180681-b125-5e41-bd04-3c8b046175b4");
}

TEST(BoostUUID, actualOsmNode)
{
  uuid x = fromURL("http://openstreetmap.org/node/1");
  uuid y = fromURL("http://openstreetmap.org/node/152370223");
  EXPECT_NE(x, y);
  // MUST yield same result as Python fromURL() function:
  EXPECT_EQ(toHexString(y), "8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac");
}

TEST(BoostUUID, fromHexString)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string r = toHexString(fromHexString(s));
  EXPECT_EQ(s, r);
}

TEST(BoostUUID, fromStringNoDashes)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string s_hex("da7c242f2efe5175996149cc621b80b9");
  std::string r = toHexString(fromHexString(s_hex));
  EXPECT_EQ(s, r);
}

TEST(BoostUUID, fromBracesString)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string s_braces = "{" + s + "}";
  std::string r = toHexString(fromHexString(s_braces));
  EXPECT_EQ(s, r);
}

TEST(BoostUUID, fromUrnString)
{
  // This documents boost 1.46.1 behavior, but is an undefined
  // fromHexString() input, not really a valid test case.
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string s_urn = "urn:uuid:" + s;
  std::string r = toHexString(fromHexString(s_urn));
  EXPECT_NE(s, r);
}

TEST(BoostUUID, fromTooLongString)
{
  // This documents boost 1.46.1 behavior, but is an undefined
  // fromHexString() input, not really a valid test case.
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string s_too_long = s + "-0001";
  std::string r = toHexString(fromHexString(s_too_long));
  EXPECT_EQ(s, r);
}

TEST(BoostUUID, fromTooShortString)
{
  // This documents boost 1.46.1 behavior, but is an undefined
  // fromHexString() input, not really a valid test case.
  std::string s("da7c242f-2efe-5175-9961-49cc621b80");
  try
    {
      uuid x = fromHexString(s);
      FAIL();                           // expected exception not thrown
      EXPECT_NE(toHexString(x), s);
    }
  catch (std::runtime_error &e)
    {
      EXPECT_EQ(e.what(), std::string("invalid uuid string"));
    }
  catch (...)
    {
      FAIL();                           // unexpected exception
    }
}

TEST(BoostUUID, fromBogusString)
{
  // This documents boost 1.46.1 behavior, but is an undefined
  // fromHexString() input, not really a valid test case.
  std::string s("Invalid UUID string");
  try
    {
      uuid x = fromHexString(s);
      FAIL();                           // expected exception not thrown
      EXPECT_NE(toHexString(x), s);
    }
  catch (std::runtime_error &e)
    {
      EXPECT_EQ(e.what(), std::string("invalid uuid string"));
    }
  catch (...)
    {
      FAIL();                           // unexpected exception
    }
}

TEST(UniqueID, nilMessage)
{
  UniqueID x;
  UniqueID y = toMsg(uuid());
  EXPECT_EQ(x.uuid, y.uuid);
}

TEST(UniqueID, randomMessage)
{
  UniqueID x;
  UniqueID y = toMsg(fromRandom());
  EXPECT_NE(x.uuid, y.uuid);
}

TEST(UniqueID, equivalentMessages)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  UniqueID x = toMsg(fromHexString(s));
  UniqueID y = toMsg(fromHexString(s));
  EXPECT_EQ(x.uuid, y.uuid);
  EXPECT_EQ(s, toHexString(y));
}

TEST(UniqueID, toAndFromMessage)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  uuid x = uuid(fromHexString(s));
  uuid y = fromMsg(toMsg(x));
  EXPECT_EQ(x, y);
}

TEST(UniqueID, messageToString)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  UniqueID x = toMsg(fromHexString(s));
  std::string y = toHexString(x);
  EXPECT_EQ(s, y);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
