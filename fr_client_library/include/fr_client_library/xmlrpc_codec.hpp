// XML-RPC value model and codec for the controller's command channel (TCP 20003).
//
// Requests are encoded the way the SDK's XmlRpc++ 0.7 does (docs/sdk_quirks.md C6): ints as
// <i4>, doubles as "%f" (six decimals), strings untagged, no <params> when there are none.
// The decoder accepts what the controller's server (xmlrpc-c Abyss) returns.
#pragma once

#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace fr_client_library
{
class XmlRpcValue
{
public:
  using Array = std::vector<XmlRpcValue>;

  XmlRpcValue(int v) : v_(std::in_place_type<int>, v) {}
  XmlRpcValue(bool v) : v_(std::in_place_type<bool>, v) {}
  XmlRpcValue(double v) : v_(std::in_place_type<double>, v) {}
  XmlRpcValue(const char * v) : v_(std::in_place_type<std::string>, v) {}
  XmlRpcValue(std::string v) : v_(std::in_place_type<std::string>, std::move(v)) {}
  XmlRpcValue(Array v) : v_(std::in_place_type<Array>, std::move(v)) {}

  bool isInt() const {return std::holds_alternative<int>(v_);}
  bool isBool() const {return std::holds_alternative<bool>(v_);}
  bool isDouble() const {return std::holds_alternative<double>(v_);}
  bool isString() const {return std::holds_alternative<std::string>(v_);}
  bool isArray() const {return std::holds_alternative<Array>(v_);}

  // Check the type first; a wrong access throws std::bad_variant_access.
  int asInt() const {return std::get<int>(v_);}
  bool asBool() const {return std::get<bool>(v_);}
  double asDouble() const {return std::get<double>(v_);}
  const std::string & asString() const {return std::get<std::string>(v_);}
  const Array & asArray() const {return std::get<Array>(v_);}

private:
  std::variant<int, bool, double, std::string, Array> v_;
};

// Throws std::invalid_argument for a non-finite double (never send NaN/inf to a robot).
std::string encodeMethodCall(const std::string & method, const std::vector<XmlRpcValue> & params);

struct XmlRpcResponse
{
  bool is_fault = false;
  XmlRpcValue value{0};      // valid when !is_fault
  int fault_code = 0;        // valid when is_fault
  std::string fault_string;  // valid when is_fault
};

// nullopt if the document is not a well-formed response of the supported subset.
std::optional<XmlRpcResponse> decodeMethodResponse(const std::string & xml);

}  // namespace fr_client_library
