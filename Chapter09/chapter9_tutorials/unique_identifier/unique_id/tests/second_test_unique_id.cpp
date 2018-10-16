//
// A second C++ unit test for unique_id interface, which must be able
// to be included in more than one source file.
//

#include <gtest/gtest.h>

#include <unique_id/unique_id.h>

using namespace unique_id;
typedef boost::uuids::uuid uuid;
typedef uuid_msgs::UniqueID UniqueID;

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test random generator
TEST(BoostUUID2, fromRandom)
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

TEST(BoostUUID2, emptyURL)
{
  std::string s;
  uuid x = fromURL(s);
  uuid y = fromURL(s);
  EXPECT_EQ(x, y);
  // MUST yield same result as Python fromURL() function:
  EXPECT_EQ(toHexString(x), "1b4db7eb-4057-5ddf-91e0-36dec72071f5");
}

TEST(BoostUUID2, sameURL)
{
  std::string s("http://openstreetmap.org/node/1");
  uuid x = fromURL(s);
  uuid y = fromURL(s);
  EXPECT_EQ(x, y);
  // MUST yield same result as Python fromURL() function:
  EXPECT_EQ(toHexString(x), "ef362ac8-9659-5481-b954-88e9b741c8f9");
}

TEST(BoostUUID2, differentOsmNamespace)
{
  uuid x = fromURL("http://openstreetmap.org/node/1");
  uuid y = fromURL("http://openstreetmap.org/way/1");
  EXPECT_NE(x, y);
  // MUST yield same result as Python fromURL() function:
  EXPECT_EQ(toHexString(y), "b3180681-b125-5e41-bd04-3c8b046175b4");
}

TEST(BoostUUID2, actualOsmNode)
{
  uuid x = fromURL("http://openstreetmap.org/node/1");
  uuid y = fromURL("http://openstreetmap.org/node/152370223");
  EXPECT_NE(x, y);
  // MUST yield same result as Python fromURL() function:
  EXPECT_EQ(toHexString(y), "8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac");
}

TEST(BoostUUID2, fromHexString)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string r = toHexString(fromHexString(s));
  EXPECT_EQ(s, r);
}

TEST(BoostUUID2, fromStringNoDashes)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string s_hex("da7c242f2efe5175996149cc621b80b9");
  std::string r = toHexString(fromHexString(s_hex));
  EXPECT_EQ(s, r);
}

TEST(BoostUUID2, fromBracesString)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string s_braces = "{" + s + "}";
  std::string r = toHexString(fromHexString(s_braces));
  EXPECT_EQ(s, r);
}

TEST(BoostUUID2, fromUrnString)
{
  // This documents boost 1.46.1 behavior, but is an undefined
  // fromHexString() input, not really a valid test case.
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string s_urn = "urn:uuid:" + s;
  std::string r = toHexString(fromHexString(s_urn));
  EXPECT_NE(s, r);
}

TEST(BoostUUID2, fromTooLongString)
{
  // This documents boost 1.46.1 behavior, but is an undefined
  // fromHexString() input, not really a valid test case.
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  std::string s_too_long = s + "-0001";
  std::string r = toHexString(fromHexString(s_too_long));
  EXPECT_EQ(s, r);
}

TEST(BoostUUID2, fromTooShortString)
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

TEST(BoostUUID2, fromBogusString)
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

TEST(UniqueID2, nilMessage)
{
  UniqueID x;
  UniqueID y = toMsg(uuid());
  EXPECT_EQ(x.uuid, y.uuid);
}

TEST(UniqueID2, randomMessage)
{
  UniqueID x;
  UniqueID y = toMsg(fromRandom());
  EXPECT_NE(x.uuid, y.uuid);
}

TEST(UniqueID2, equivalentMessages)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  UniqueID x = toMsg(fromHexString(s));
  UniqueID y = toMsg(fromHexString(s));
  EXPECT_EQ(x.uuid, y.uuid);
  EXPECT_EQ(s, toHexString(y));
}

TEST(UniqueID2, toAndFromMessage)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  uuid x = uuid(fromHexString(s));
  uuid y = fromMsg(toMsg(x));
  EXPECT_EQ(x, y);
}

TEST(UniqueID2, messageToString)
{
  std::string s("da7c242f-2efe-5175-9961-49cc621b80b9");
  UniqueID x = toMsg(fromHexString(s));
  std::string y = toHexString(x);
  EXPECT_EQ(s, y);
}
