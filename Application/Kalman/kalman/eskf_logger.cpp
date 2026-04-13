// ESKF Logger Implementation
// Part of Phase 1: Standalone Core ESKF Library

#include "eskf_logger.hpp"

namespace eskf {

// ============================================================
// Global Logger Instance
// ============================================================

namespace {
  NullEskfLogger null_logger_;
  IEskfLogger* current_logger_ = &null_logger_;
}

IEskfLogger& getEskfLogger() {
  return *current_logger_;
}

void setEskfLogger(IEskfLogger* logger) {
  if (logger != nullptr) {
    current_logger_ = logger;
  } else {
    current_logger_ = &null_logger_;
  }
}

} // namespace eskf

