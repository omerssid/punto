#include "core/parallel.hpp"

#include <cstdlib>
#include <charconv>
#include <string_view>

namespace punto {
namespace {

// Below this, thread-spawn overhead outweighs any gain.
constexpr std::size_t kMinItemsPerWorker = 4096;

std::vector<std::pair<std::size_t, std::size_t>> splitRanges(std::size_t n, unsigned workers) {
  std::vector<std::pair<std::size_t, std::size_t>> out;
  out.reserve(workers);
  const std::size_t base = n / workers;
  const std::size_t rem = n % workers;
  std::size_t cursor = 0;
  for (unsigned w = 0; w < workers; ++w) {
    // Spread the remainder across the first `rem` workers rather than
    // dumping it all on the last one.
    const std::size_t count = base + (w < rem ? 1 : 0);
    out.emplace_back(cursor, cursor + count);
    cursor += count;
  }
  return out;
}

unsigned clampWorkers(std::size_t n, unsigned workers) {
  if (workers == 0) workers = 1;
  const std::size_t byWork = std::max<std::size_t>(1, n / kMinItemsPerWorker);
  return static_cast<unsigned>(std::min<std::size_t>(workers, byWork));
}

}  // namespace

unsigned defaultWorkerCount() {
  if (const char* env = std::getenv("PUNTO_THREADS")) {
    unsigned v = 0;
    const std::string_view sv(env);
    if (std::from_chars(sv.data(), sv.data() + sv.size(), v).ec == std::errc{} && v > 0) return v;
  }
  const unsigned hw = std::thread::hardware_concurrency();
  return hw > 0 ? hw : 4;
}

void parallelRanges(std::size_t n, unsigned workers,
                    const std::function<void(std::size_t, std::size_t, unsigned)>& body) {
  if (n == 0) return;
  workers = clampWorkers(n, workers);
  if (workers == 1) {
    body(0, n, 0);
    return;
  }
  const auto ranges = splitRanges(n, workers);
  std::vector<std::jthread> threads;
  threads.reserve(workers - 1);
  // Worker 0 runs on the calling thread so we spawn one fewer.
  for (unsigned w = 1; w < workers; ++w) {
    threads.emplace_back([&, w] { body(ranges[w].first, ranges[w].second, w); });
  }
  body(ranges[0].first, ranges[0].second, 0);
  // ~jthread joins.
}

void parallelRangesCancellable(
    std::size_t n, unsigned workers, std::stop_token stop,
    const std::function<void(std::size_t, std::size_t, unsigned, const std::stop_token&)>& body) {
  if (n == 0) return;
  workers = clampWorkers(n, workers);
  if (workers == 1) {
    body(0, n, 0, stop);
    return;
  }
  const auto ranges = splitRanges(n, workers);
  std::vector<std::jthread> threads;
  threads.reserve(workers - 1);
  for (unsigned w = 1; w < workers; ++w) {
    threads.emplace_back([&, w] { body(ranges[w].first, ranges[w].second, w, stop); });
  }
  body(ranges[0].first, ranges[0].second, 0, stop);
}

}  // namespace punto
