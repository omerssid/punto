#include "scene/radix_sort.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

#include "core/parallel.hpp"

namespace pcv {
namespace {

constexpr int kRadixBits = 8;
constexpr std::size_t kBuckets = 1u << kRadixBits;
constexpr std::uint64_t kRadixMask = kBuckets - 1;

// Per-worker histograms, padded so neighbouring workers never share a cache
// line and ping-pong it between cores.
struct alignas(64) Histogram {
  std::uint32_t counts[kBuckets];
};

}  // namespace

void radixSortByBits(std::uint64_t* keys, std::uint64_t* scratch, std::size_t count,
                     int firstBit, int bitCount, unsigned workers) {
  if (count < 2 || bitCount <= 0) return;
  if (workers == 0) workers = defaultWorkerCount();

  std::uint64_t* src = keys;
  std::uint64_t* dst = scratch;

  std::vector<Histogram> histograms(workers);
  std::vector<std::array<std::uint32_t, kBuckets>> offsets(workers);

  const int lastBit = firstBit + bitCount;
  for (int shift = firstBit; shift < lastBit; shift += kRadixBits) {
    for (Histogram& h : histograms) std::memset(h.counts, 0, sizeof(h.counts));

    parallelRanges(count, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
      std::uint32_t* counts = histograms[w].counts;
      for (std::size_t i = begin; i < end; ++i)
        ++counts[(src[i] >> shift) & kRadixMask];
    });

    // Exclusive prefix sum over (bucket, worker), so every worker knows
    // exactly where its share of each bucket starts and can scatter without
    // any synchronisation.
    std::uint32_t running = 0;
    for (std::size_t bucket = 0; bucket < kBuckets; ++bucket) {
      for (unsigned w = 0; w < workers; ++w) {
        offsets[w][bucket] = running;
        running += histograms[w].counts[bucket];
      }
    }

    parallelRanges(count, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
      auto cursor = offsets[w];
      for (std::size_t i = begin; i < end; ++i) {
        const std::uint64_t key = src[i];
        dst[cursor[(key >> shift) & kRadixMask]++] = key;
      }
    });

    std::swap(src, dst);
  }

  // An odd number of passes leaves the result in the scratch buffer.
  if (src != keys) std::memcpy(keys, src, count * sizeof(std::uint64_t));
}

}  // namespace pcv
