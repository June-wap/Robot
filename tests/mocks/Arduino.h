#ifndef TEST_MOCK_ARDUINO_H
#define TEST_MOCK_ARDUINO_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string>

#define PROGMEM
#define IRAM_ATTR
#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1
#define CHANGE 3
#define SERIAL_8N1 0

extern unsigned long gMockMillis;

inline float radians(float d) { return d * 0.0174532925f; }
inline float degrees(float r) { return r * 57.2957795f; }
inline unsigned long millis() { return gMockMillis; }
inline uint32_t micros() { return (uint32_t)(gMockMillis * 1000UL); }
inline void delay(unsigned long ms) { gMockMillis += ms; }
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int  digitalRead(int) { return 0; }
inline int  digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void noInterrupts() {}
inline void interrupts() {}

class String {
  std::string s_;
public:
  String() {}
  String(const char* c) : s_(c ? c : "") {}
  String(int v) : s_(std::to_string(v)) {}
  String(unsigned int v) : s_(std::to_string(v)) {}
  const char* c_str() const { return s_.c_str(); }
  size_t length() const { return s_.size(); }
  void reserve(size_t n) { s_.reserve(n); }
  int toInt() const { return atoi(s_.c_str()); }
  float toFloat() const { return strtof(s_.c_str(), nullptr); }
  void replace(const char* a, const char* b) {
    if (!a || !*a) return;
    std::string from(a), to(b ? b : "");
    size_t pos = 0;
    while ((pos = s_.find(from, pos)) != std::string::npos) {
      s_.replace(pos, from.length(), to);
      pos += to.length();
    }
  }
  bool operator==(const char* c) const { return s_ == (c ? c : ""); }
  String& operator+=(const char* c) { if (c) s_ += c; return *this; }
  String& operator+=(char c) { s_ += c; return *this; }
  String& operator+=(int v) { s_ += std::to_string(v); return *this; }
  String& operator+=(uint16_t v) { s_ += std::to_string(v); return *this; }
  String operator+(const String& o) const { String r(*this); r.s_ += o.s_; return r; }
  friend String operator+(const char* a, const String& b) { String r(a); r.s_ += b.s_; return r; }
};

struct HardwareSerial {
  void begin(unsigned long, uint32_t = 0, int = -1, int = -1) {}
  void end() {}
  void setRxBufferSize(size_t) {}
  int  available() { return 0; }
  int  read() { return 0; }
  void print(const char*) {}
  void print(char) {}
  void println(const char* = "") {}
  void println(const String&) {}
  void printf(const char*, ...) {}
  void flush() {}
};

extern HardwareSerial Serial, Serial1, Serial2;

#endif // TEST_MOCK_ARDUINO_H
