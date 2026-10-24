#include "core/log.hpp"

#include <chrono>
#include <mutex>

namespace punto {
namespace {

LogLevel g_level = LogLevel::Info;
std::mutex g_mutex;

const char* levelTag(LogLevel level) {
  switch (level) {
    case LogLevel::Debug: return "\x1b[90mdebug\x1b[0m";
    case LogLevel::Info:  return "\x1b[36minfo \x1b[0m";
    case LogLevel::Warn:  return "\x1b[33mwarn \x1b[0m";
    case LogLevel::Error: return "\x1b[31merror\x1b[0m";
  }
  return "?????";
}

long long nowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

void setLogLevel(LogLevel level) { g_level = level; }

void logRaw(LogLevel level, std::string_view msg) {
  if (level < g_level) return;
  std::lock_guard lock(g_mutex);
  std::fprintf(level >= LogLevel::Warn ? stderr : stdout, "[%s] %.*s\n", levelTag(level),
               static_cast<int>(msg.size()), msg.data());
}

Stopwatch::Stopwatch() : startNs_(nowNs()) {}
double Stopwatch::elapsedMs() const { return static_cast<double>(nowNs() - startNs_) * 1e-6; }
void Stopwatch::restart() { startNs_ = nowNs(); }

ScopedTimer::ScopedTimer(std::string_view label) : label_(label), startNs_(nowNs()) {}

double ScopedTimer::elapsedMs() const { return static_cast<double>(nowNs() - startNs_) * 1e-6; }

ScopedTimer::~ScopedTimer() { logDebug("{}: {:.1f} ms", label_, elapsedMs()); }

}  // namespace punto
