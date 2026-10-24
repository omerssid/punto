#pragma once

#include <cstdio>
#include <format>
#include <string_view>
#include <utility>

namespace punto {

enum class LogLevel { Debug, Info, Warn, Error };

void logRaw(LogLevel level, std::string_view msg);

template <typename... Args>
void logDebug(std::format_string<Args...> fmt, Args&&... args) {
  logRaw(LogLevel::Debug, std::format(fmt, std::forward<Args>(args)...));
}
template <typename... Args>
void logInfo(std::format_string<Args...> fmt, Args&&... args) {
  logRaw(LogLevel::Info, std::format(fmt, std::forward<Args>(args)...));
}
template <typename... Args>
void logWarn(std::format_string<Args...> fmt, Args&&... args) {
  logRaw(LogLevel::Warn, std::format(fmt, std::forward<Args>(args)...));
}
template <typename... Args>
void logError(std::format_string<Args...> fmt, Args&&... args) {
  logRaw(LogLevel::Error, std::format(fmt, std::forward<Args>(args)...));
}

void setLogLevel(LogLevel level);

// Silent wall-clock timer. Used for the load-phase breakdown, where the
// numbers go into LoadTiming rather than the log.
class Stopwatch {
 public:
  Stopwatch();
  double elapsedMs() const;
  void restart();

 private:
  long long startNs_;
};

// Scoped wall-clock timer; logs "<label>: 12.3 ms" when it goes out of scope.
// Used to keep the load-time breakdown honest and visible.
class ScopedTimer {
 public:
  explicit ScopedTimer(std::string_view label);
  ~ScopedTimer();
  double elapsedMs() const;

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

 private:
  std::string_view label_;
  long long startNs_;
};

}  // namespace punto
