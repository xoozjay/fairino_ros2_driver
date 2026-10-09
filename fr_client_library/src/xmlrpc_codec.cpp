#include "fr_client_library/xmlrpc_codec.hpp"

#include <charconv>
#include <cmath>
#include <stdexcept>

namespace fr_client_library
{
namespace
{
std::string xmlEncode(const std::string & raw)
{
  std::string out;
  for (const char c : raw) {
    switch (c) {
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '&': out += "&amp;"; break;
      case '\'': out += "&apos;"; break;
      case '"': out += "&quot;"; break;
      default: out += c;
    }
  }
  return out;
}

std::string xmlDecode(const std::string & s)
{
  static const struct {const char * entity; char raw;} kEntities[] = {
    {"&lt;", '<'}, {"&gt;", '>'}, {"&amp;", '&'}, {"&apos;", '\''}, {"&quot;", '"'}};
  std::string out;
  for (std::size_t i = 0; i < s.size(); ++i) {
    bool replaced = false;
    if (s[i] == '&') {
      for (const auto & e : kEntities) {
        const std::string ent = e.entity;
        if (s.compare(i, ent.size(), ent) == 0) {
          out += e.raw;
          i += ent.size() - 1;
          replaced = true;
          break;
        }
      }
    }
    if (!replaced) {
      out += s[i];
    }
  }
  return out;
}

std::string encodeValue(const XmlRpcValue & v)
{
  if (v.isInt()) {
    return "<value><i4>" + std::to_string(v.asInt()) + "</i4></value>";
  }
  if (v.isBool()) {
    return std::string("<value><boolean>") + (v.asBool() ? "1" : "0") + "</boolean></value>";
  }
  if (v.isDouble()) {
    if (!std::isfinite(v.asDouble())) {
      throw std::invalid_argument("non-finite double in XML-RPC parameter");
    }
    char buf[400];  // locale independent, same text as printf("%f")
    const auto r = std::to_chars(buf, buf + sizeof(buf), v.asDouble(), std::chars_format::fixed, 6);
    return "<value><double>" + std::string(buf, r.ptr) + "</double></value>";
  }
  if (v.isString()) {
    return "<value>" + xmlEncode(v.asString()) + "</value>";
  }
  std::string xml = "<value><array><data>";
  for (const auto & e : v.asArray()) {
    xml += encodeValue(e);
  }
  return xml + "</data></array></value>";
}

// ---- decoder ----

constexpr int kMaxDepth = 16;

class Reader
{
public:
  explicit Reader(const std::string & s) : s_(s) {}

  bool atEnd() {skipWs(); return p_ >= s_.size();}

  bool peek(const char * lit)
  {
    skipWs();
    return s_.compare(p_, std::char_traits<char>::length(lit), lit) == 0;
  }

  bool consume(const char * lit)
  {
    if (!peek(lit)) {
      return false;
    }
    p_ += std::char_traits<char>::length(lit);
    return true;
  }

  bool skipProlog()
  {
    if (peek("<?xml")) {
      const auto end = s_.find("?>", p_);
      if (end == std::string::npos) {
        return false;
      }
      p_ = end + 2;
    }
    return true;
  }

  // Raw text up to the next '<' (whitespace is significant), entities decoded.
  std::string text()
  {
    const auto end = s_.find('<', p_);
    const std::string raw = s_.substr(p_, end == std::string::npos ? std::string::npos : end - p_);
    p_ = end == std::string::npos ? s_.size() : end;
    return xmlDecode(raw);
  }

  std::optional<XmlRpcValue> value(int depth = 0)
  {
    if (depth > kMaxDepth || !consume("<value>")) {
      return std::nullopt;
    }
    const std::size_t start = p_;  // untagged strings keep their leading whitespace
    std::optional<XmlRpcValue> v;
    if (peek("<i4>") || peek("<int>")) {
      const bool i4 = peek("<i4>");
      consume(i4 ? "<i4>" : "<int>");
      const std::string t = text();
      int n = 0;
      const auto r = std::from_chars(t.data(), t.data() + t.size(), n);
      if (t.empty() || r.ec != std::errc() || r.ptr != t.data() + t.size() ||
        !consume(i4 ? "</i4>" : "</int>"))
      {
        return std::nullopt;
      }
      v = XmlRpcValue(n);
    } else if (consume("<boolean>")) {
      const std::string t = text();
      if ((t != "0" && t != "1") || !consume("</boolean>")) {
        return std::nullopt;
      }
      v = XmlRpcValue(t == "1");
    } else if (consume("<double>")) {
      const std::string t = text();
      double d = 0;
      const auto r = std::from_chars(t.data(), t.data() + t.size(), d);
      if (t.empty() || r.ec != std::errc() || r.ptr != t.data() + t.size() ||
        !consume("</double>"))
      {
        return std::nullopt;
      }
      v = XmlRpcValue(d);
    } else if (consume("<string/>")) {
      v = XmlRpcValue(std::string());
    } else if (consume("<string>")) {
      std::string t = text();
      if (!consume("</string>")) {
        return std::nullopt;
      }
      v = XmlRpcValue(std::move(t));
    } else if (consume("<array>")) {
      XmlRpcValue::Array items;
      if (!consume("<data>")) {
        return std::nullopt;
      }
      while (peek("<value>")) {
        auto item = value(depth + 1);
        if (!item) {
          return std::nullopt;
        }
        items.push_back(std::move(*item));
      }
      if (!consume("</data>") || !consume("</array>")) {
        return std::nullopt;
      }
      v = XmlRpcValue(std::move(items));
    } else if (peek("<") && !peek("</value>")) {
      return std::nullopt;  // struct, dateTime, base64, ...: not supported
    } else {
      p_ = start;
      v = XmlRpcValue(text());  // untagged value is a string
    }
    if (!consume("</value>")) {
      return std::nullopt;
    }
    return v;
  }

  // <fault><value><struct><member><name>..</name><value>..</value></member>...</struct></value></fault>
  bool fault(int & code, std::string & message)
  {
    if (!consume("<fault>") || !consume("<value>") || !consume("<struct>")) {
      return false;
    }
    bool have_code = false;
    while (consume("<member>")) {
      if (!consume("<name>")) {
        return false;
      }
      const std::string name = text();
      if (!consume("</name>")) {
        return false;
      }
      const auto v = value();
      if (!v || !consume("</member>")) {
        return false;
      }
      if (name == "faultCode" && v->isInt()) {
        code = v->asInt();
        have_code = true;
      } else if (name == "faultString" && v->isString()) {
        message = v->asString();
      }
    }
    return have_code && consume("</struct>") && consume("</value>") && consume("</fault>");
  }

private:
  void skipWs()
  {
    while (p_ < s_.size() && (s_[p_] == ' ' || s_[p_] == '\t' || s_[p_] == '\r' || s_[p_] == '\n')) {
      ++p_;
    }
  }

  const std::string & s_;
  std::size_t p_ = 0;
};
}  // namespace

std::string encodeMethodCall(const std::string & method, const std::vector<XmlRpcValue> & params)
{
  std::string xml = "<?xml version=\"1.0\"?>\r\n<methodCall><methodName>" + method +
    "</methodName>\r\n";
  if (!params.empty()) {
    xml += "<params>";
    for (const auto & p : params) {
      xml += "<param>" + encodeValue(p) + "</param>";
    }
    xml += "</params>";
  }
  return xml + "</methodCall>\r\n";
}

std::optional<XmlRpcResponse> decodeMethodResponse(const std::string & xml)
{
  Reader r(xml);
  XmlRpcResponse response;
  if (!r.skipProlog() || !r.consume("<methodResponse>")) {
    return std::nullopt;
  }
  if (r.peek("<fault>")) {
    response.is_fault = true;
    if (!r.fault(response.fault_code, response.fault_string)) {
      return std::nullopt;
    }
  } else {
    if (!r.consume("<params>") || !r.consume("<param>")) {
      return std::nullopt;
    }
    auto v = r.value();
    if (!v || !r.consume("</param>") || !r.consume("</params>")) {
      return std::nullopt;
    }
    response.value = std::move(*v);
  }
  if (!r.consume("</methodResponse>") || !r.atEnd()) {
    return std::nullopt;
  }
  return response;
}

}  // namespace fr_client_library
