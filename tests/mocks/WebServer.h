#ifndef TEST_MOCK_WEBSERVER_H
#define TEST_MOCK_WEBSERVER_H

#include "Arduino.h"
#include <map>
#include <string>

struct WiFiClient {
  std::string written;
  size_t write(const uint8_t* p, size_t n) {
    written.append((const char*)p, n);
    return n;
  }
};

class WebServer {
public:
  int lastStatus = 0;
  std::string lastContentType;
  std::string lastBody;
  std::map<std::string, std::string> args;
  WiFiClient mockClient;

  WebServer(int) {}
  void on(const char*, void (*)()) {}
  void begin() {}
  void handleClient() {}
  void sendHeader(const char*, const char*) {}
  void setContentLength(size_t) {}
  void send(int code, const char* contentType, const char* body) {
    lastStatus = code;
    lastContentType = contentType ? contentType : "";
    lastBody = body ? body : "";
  }
  void send(int code, const char* contentType, const String& body) {
    send(code, contentType, body.c_str());
  }
  bool hasArg(const char* k) const {
    return k && args.find(k) != args.end();
  }
  String arg(const char* k) const {
    if (!k) return "";
    auto it = args.find(k);
    return (it != args.end()) ? String(it->second.c_str()) : String("");
  }
  WiFiClient& client() { return mockClient; }
};

#endif // TEST_MOCK_WEBSERVER_H
