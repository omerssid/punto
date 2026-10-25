#include "core/memory.hpp"

#include <cstdio>
#include <cstring>

namespace punto {
namespace {

// Reads one "Key:   12345 kB" line out of /proc/meminfo.
std::uint64_t readMeminfoKB(const char* key) {
  std::FILE* f = std::fopen("/proc/meminfo", "rb");
  if (f == nullptr) return 0;

  const std::size_t keyLen = std::strlen(key);
  unsigned long long valueKB = 0;
  char line[256];
  while (std::fgets(line, sizeof(line), f) != nullptr) {
    if (std::strncmp(line, key, keyLen) != 0 || line[keyLen] != ':') continue;
    if (std::sscanf(line + keyLen + 1, "%llu", &valueKB) != 1) valueKB = 0;
    break;
  }
  std::fclose(f);
  return static_cast<std::uint64_t>(valueKB);
}

}  // namespace

std::uint64_t availableMemoryBytes() { return readMeminfoKB("MemAvailable") * 1024; }

std::uint64_t totalMemoryBytes() { return readMeminfoKB("MemTotal") * 1024; }

void preferSelfForOomKill() {
  // 1000 is the maximum: it makes the kernel pick this process ahead of
  // anything running at a lower adjustment, regardless of relative size.
  if (std::FILE* f = std::fopen("/proc/self/oom_score_adj", "we")) {
    std::fputs("1000\n", f);
    std::fclose(f);
  }
}

}  // namespace punto
