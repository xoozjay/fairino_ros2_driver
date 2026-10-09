#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "fr_client_library/xmlrpc_codec.hpp"

using namespace fr_client_library;

// ---- encoding: byte for byte what XmlRpc++ 0.7 produces (XmlRpcClient/XmlRpcValue.cpp) ----

TEST(Encode, NoParamsOmitsParamsElement)
{
  EXPECT_EQ(
    encodeMethodCall("GetSoftwareVersion", {}),
    "<?xml version=\"1.0\"?>\r\n<methodCall><methodName>GetSoftwareVersion</methodName>\r\n"
    "</methodCall>\r\n");
}

TEST(Encode, ScalarTypes)
{
  EXPECT_EQ(
    encodeMethodCall("Foo", {1, -2, 2.5, true, false, "a<b&c'\">"}),
    "<?xml version=\"1.0\"?>\r\n<methodCall><methodName>Foo</methodName>\r\n<params>"
    "<param><value><i4>1</i4></value></param>"
    "<param><value><i4>-2</i4></value></param>"
    "<param><value><double>2.500000</double></value></param>"
    "<param><value><boolean>1</boolean></value></param>"
    "<param><value><boolean>0</boolean></value></param>"
    "<param><value>a&lt;b&amp;c&apos;&quot;&gt;</value></param>"
    "</params></methodCall>\r\n");
}

TEST(Encode, DoubleUsesSixDecimalsLikePrintfF)
{
  const auto x = encodeMethodCall("F", {XmlRpcValue(-12.3456789), XmlRpcValue(0.0000004)});
  EXPECT_NE(x.find("<double>-12.345679</double>"), std::string::npos);
  EXPECT_NE(x.find("<double>0.000000</double>"), std::string::npos);
}

TEST(Encode, Array)
{
  EXPECT_EQ(
    encodeMethodCall("F", {XmlRpcValue(XmlRpcValue::Array{1.0, 2})}),
    "<?xml version=\"1.0\"?>\r\n<methodCall><methodName>F</methodName>\r\n<params><param>"
    "<value><array><data><value><double>1.000000</double></value><value><i4>2</i4></value>"
    "</data></array></value></param></params></methodCall>\r\n");
}

TEST(Encode, NonFiniteDoubleThrows)
{
  EXPECT_THROW(encodeMethodCall("F", {std::nan("")}), std::invalid_argument);
  EXPECT_THROW(
    encodeMethodCall("F", {std::numeric_limits<double>::infinity()}), std::invalid_argument);
}

// ---- decoding ----

TEST(Decode, SimulatorGetSoftwareVersion)  // recorded from the 3.8.3 simulator
{
  const std::string xml =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<methodResponse>\r\n<params>\r\n"
    "<param><value><array><data>\r\n<value><i4>0</i4></value>\r\n"
    "<value><string>FR5-V1-002(V6.0)</string></value>\r\n<value><string>v3.8.3</string></value>\r\n"
    "<value><string>V3.8.8-LA</string></value>\r\n</data></array></value></param>\r\n"
    "</params>\r\n</methodResponse>\r\n";
  const auto r = decodeMethodResponse(xml);
  ASSERT_TRUE(r.has_value());
  ASSERT_FALSE(r->is_fault);
  ASSERT_TRUE(r->value.isArray());
  const auto & a = r->value.asArray();
  ASSERT_EQ(a.size(), 4u);
  EXPECT_EQ(a[0].asInt(), 0);
  EXPECT_EQ(a[1].asString(), "FR5-V1-002(V6.0)");
  EXPECT_EQ(a[3].asString(), "V3.8.8-LA");
}

TEST(Decode, ScalarsAndUntaggedStrings)
{
  const auto wrap = [](const std::string & v) {
      return "<methodResponse><params><param>" + v + "</param></params></methodResponse>";
    };
  EXPECT_EQ(decodeMethodResponse(wrap("<value><i4>-7</i4></value>"))->value.asInt(), -7);
  EXPECT_EQ(decodeMethodResponse(wrap("<value><int>8</int></value>"))->value.asInt(), 8);
  EXPECT_TRUE(decodeMethodResponse(wrap("<value><boolean>1</boolean></value>"))->value.asBool());
  EXPECT_DOUBLE_EQ(
    decodeMethodResponse(wrap("<value><double>-1.5e2</double></value>"))->value.asDouble(), -150.0);
  EXPECT_EQ(decodeMethodResponse(wrap("<value> hi &lt;there&gt; </value>"))->value.asString(),
    " hi <there> ");
  EXPECT_EQ(decodeMethodResponse(wrap("<value></value>"))->value.asString(), "");
  EXPECT_EQ(decodeMethodResponse(wrap("<value><string/></value>"))->value.asString(), "");
}

TEST(Decode, NestedArray)
{
  const auto r = decodeMethodResponse(
    "<methodResponse><params><param><value><array><data><value><array><data>"
    "<value><i4>1</i4></value></data></array></value></data></array></value></param></params>"
    "</methodResponse>");
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->value.asArray()[0].asArray()[0].asInt(), 1);
}

TEST(Decode, Fault)
{
  const auto r = decodeMethodResponse(
    "<?xml version=\"1.0\"?>\r\n<methodResponse>\r\n<fault>\r\n<value><struct>\r\n"
    "<member><name>faultCode</name><value><i4>-503</i4></value></member>\r\n"
    "<member><name>faultString</name><value><string>Method 'X' not defined</string></value></member>\r\n"
    "</struct></value>\r\n</fault>\r\n</methodResponse>\r\n");
  ASSERT_TRUE(r.has_value());
  EXPECT_TRUE(r->is_fault);
  EXPECT_EQ(r->fault_code, -503);
  EXPECT_EQ(r->fault_string, "Method 'X' not defined");
}

TEST(Decode, MalformedIsRejected)
{
  for (const char * bad : {
      "", "garbage", "<methodResponse>",
      "<methodResponse><params><param><value><i4>1</i4></value></param></params>",  // truncated
      "<methodCall><params></params></methodCall>",
      "<methodResponse><params><param><value><i4>x</i4></value></param></params></methodResponse>",
      "<methodResponse><params><param><value><boolean>2</boolean></value></param></params></methodResponse>",
      "<methodResponse><params><param><value><struct></struct></value></param></params></methodResponse>",
      "<methodResponse><params><param><value><i4>1</i4></value></param></params></methodResponse>junk",
      "<methodResponse><fault><value><struct></struct></value></fault></methodResponse>",
    })
  {
    EXPECT_FALSE(decodeMethodResponse(bad).has_value()) << bad;
  }
}

TEST(Decode, DeepNestingIsRejectedNotCrashed)
{
  std::string v;
  for (int i = 0; i < 200; ++i) {v += "<value><array><data>";}
  v += "<value><i4>1</i4></value>";
  for (int i = 0; i < 200; ++i) {v += "</data></array></value>";}
  EXPECT_FALSE(decodeMethodResponse(
    "<methodResponse><params><param>" + v + "</param></params></methodResponse>").has_value());
}
