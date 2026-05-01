#pragma once

#include <stdint.h>

namespace hal {

struct ILogSink {
  virtual ~ILogSink() = default;
  virtual void print(const char* s) = 0;
  virtual void println(const char* s) = 0;
  virtual void printf(const char* fmt, ...) = 0;
};

}  // namespace hal
