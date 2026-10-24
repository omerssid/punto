// Chunk construction via Morton-order sorting.
//
// The earlier version partitioned an octree top-down, making roughly seven
// passes over sixteen bytes per point and touching each element three times
// per level -- about 336 bytes of memory traffic per point. Sorting by
// Morton code instead costs one key-building pass plus four radix passes
// over eight-byte keys, after which every octree node is simply a
// contiguous range of the sorted array, found by binary search.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

#include "core/large_buffer.hpp"
#include "core/log.hpp"
#include "core/parallel.hpp"
#include "scene/chunk.hpp"
#include "scene/morton.hpp"
#include "scene/radix_sort.hpp"

namespace punto {
namespace {

// A leaf of the linear octree: a run of the sorted key array.
struct Leaf {
  std::size_t lo = 0;
  std::size_t hi = 0;
  int level = 0;
  std::size_t count() const { return hi - lo; }
};

// Walks the sorted keys, emitting a leaf wherever a node is small enough
// and fine enough. Subdividing is a binary search for the end of each child
// prefix run, not a pass over the points, so the whole recursion costs
// about O(leaves * log n).
void emitLeaves(const std::uint64_t* keys, std::size_t lo, std::size_t hi, int level,
                int minLevel, std::uint32_t maxPoints, std::uint32_t minPointsToSplit,
                std::vector<Leaf>& out) {
  const std::size_t count = hi - lo;
  if (count == 0) return;

  const bool tooMany = count > maxPoints;
  // Nodes coarser than minLevel are split regardless of population, so a
  // sparse region cannot become one enormous chunk that never culls.
  const bool tooCoarse = level < minLevel && count > minPointsToSplit;

  if ((!tooMany && !tooCoarse) || level >= kMortonMaxLevel) {
    out.push_back(Leaf{lo, hi, level});
    return;
  }

  const int childLevel = level + 1;
  std::size_t cursor = lo;
  while (cursor < hi) {
    const std::uint64_t prefix = keyPrefix(keys[cursor], childLevel);
    // Iterating over occupied prefixes rather than all eight octants skips
    // the empty ones for free.
    const std::uint64_t* end =
        std::upper_bound(keys + cursor, keys + hi, prefix,
                         [childLevel](std::uint64_t value, std::uint64_t key) {
                           return value < keyPrefix(key, childLevel);
                         });
    const std::size_t next = static_cast<std::size_t>(end - keys);
    emitLeaves(keys, cursor, next, childLevel, minLevel, maxPoints, minPointsToSplit, out);
    cursor = next;
  }
}

template <typename T>
void gather(std::vector<T>& values, const std::vector<std::uint32_t>& perm, unsigned workers) {
  if (values.empty()) return;
  const std::size_t n = values.size();
  LargeBuffer<T> out(n);
  parallelRanges(perm.size(), workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t i = begin; i < end; ++i) out[i] = values[perm[i]];
  });
  std::memcpy(values.data(), out.get(), n * sizeof(T));
}

void gatherColumn(ScalarColumn& column, const std::vector<std::uint32_t>& perm, unsigned workers) {
  if (column.count() == 0) return;
  const std::size_t esz = column.elementSize();
  const std::size_t bytes = column.count() * esz;
  LargeBuffer<std::byte> out(bytes);
  const std::byte* src = column.data();
  std::byte* dst = out.get();
  parallelRanges(perm.size(), workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t i = begin; i < end; ++i)
      std::memcpy(dst + i * esz, src + static_cast<std::size_t>(perm[i]) * esz, esz);
  });
  std::memcpy(column.data(), out.get(), bytes);
}

// Mean spacing between neighbouring points, estimated from the chunk's
// extent and population.
//
// LiDAR points lie on surfaces rather than filling volumes, so the useful
// measure is area per point, not volume per point. Using the two largest
// box dimensions gives the area of the face the surface most likely spans.
float estimateSpacing(const Aabbf& box, std::uint32_t count) {
  if (count == 0) return 0.0f;
  Vec3f d = box.size();
  float dims[3] = {d.x, d.y, d.z};
  std::sort(dims, dims + 3);
  const float area = dims[2] * dims[1];
  if (area <= 0.0f) {
    // Degenerate box: a line, or every point coincident.
    const float extent = dims[2];
    return extent > 0.0f ? extent / static_cast<float>(count) : 0.0f;
  }
  return std::sqrt(area / static_cast<float>(count));
}

}  // namespace

std::vector<Chunk> buildChunks(PointCloud& cloud, const ChunkBuildOptions& options,
                               ChunkBuildStats* stats) {
  Stopwatch totalTimer;
  std::vector<Chunk> chunks;
  const std::size_t n = cloud.size();
  if (n == 0) return chunks;

  const unsigned workers = options.workers != 0 ? options.workers : defaultWorkerCount();
  const std::uint32_t maxPoints = std::max<std::uint32_t>(1024, options.maxPointsPerChunk);

  if (!cloud.boundsLocal.valid()) cloud.recomputeBounds();

  // ------------------------------------------------- 1. sort by Morton code
  Stopwatch partitionTimer;
  const MortonGrid grid = MortonGrid::forBounds(cloud.boundsLocal);

  float maxExtent = options.maxChunkExtent;
  if (maxExtent <= 0.0f)
    maxExtent = std::max(0.5f, length(cloud.boundsLocal.size()) / 24.0f);

  // The shallowest level whose nodes are already within the extent limit.
  int minLevel = 0;
  while (minLevel < kMortonMaxLevel && grid.nodeExtent(minLevel) > maxExtent) ++minLevel;

  LargeBuffer<std::uint64_t> keys(n);
  LargeBuffer<std::uint64_t> scratch(n);
  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t i = begin; i < end; ++i)
      keys[i] = makeMortonKey(grid.codeFor(cloud.positions[i]), static_cast<std::uint32_t>(i));
  });

  // Only the code matters for ordering; the point index riding in the low
  // 32 bits can stay in any order, so those passes are skipped.
  radixSortByBits(keys.get(), scratch.get(), n, kMortonKeyShift, kMortonCodeBits, workers);
  scratch.release();

  // ------------------------------------------------ 2. linear octree leaves
  std::vector<Leaf> leaves;
  leaves.reserve(1024);
  emitLeaves(keys.get(), 0, n, 0, minLevel, maxPoints, options.minPointsToSplit, leaves);

  // Absorb undersized leaves into a neighbour.
  //
  // A leaf holding a handful of points still costs a whole indirect draw
  // command and a cull test. Merging is restricted to siblings -- leaves
  // sharing a parent -- and only where that parent is itself within the
  // extent limit, so combining them cannot reintroduce an oversized chunk.
  const std::uint32_t minPoints = std::max<std::uint32_t>(1, maxPoints / 8);
  {
    std::vector<Leaf> merged;
    merged.reserve(leaves.size());
    for (const Leaf& leaf : leaves) {
      if (!merged.empty()) {
        Leaf& prev = merged.back();
        const int parentLevel = std::min(prev.level, leaf.level) - 1;
        const bool smallEnough = prev.count() < minPoints || leaf.count() < minPoints;
        const bool fits = prev.count() + leaf.count() <= maxPoints;
        // Only merge where the parent is itself within the extent limit.
        // Letting it exceed the limit traded 131 chunks for a slower frame
        // (0.44 -> 0.47 ms): the coarser boxes cull and pick LOD worse,
        // which costs more than the saved draw commands.
        const bool parentCompact = parentLevel >= minLevel;
        const bool siblings =
            parentLevel >= 0 &&
            keyPrefix(keys[prev.lo], parentLevel) == keyPrefix(keys[leaf.lo], parentLevel);
        if (smallEnough && fits && parentCompact && siblings && prev.hi == leaf.lo) {
          prev.hi = leaf.hi;
          prev.level = parentLevel;
          continue;
        }
      }
      merged.push_back(leaf);
    }
    leaves.swap(merged);
  }
  const double partitionMs = partitionTimer.elapsedMs();

  // ---------------------------------------------- 3. LOD order per chunk
  //
  // Shuffling the permutation (rather than the point data) means the whole
  // reorder is still a single gather pass. After this, the first N entries
  // of any chunk are a uniform random sample of that chunk, for every N --
  // which is what makes continuous LOD free.
  Stopwatch lodTimer;
  std::vector<std::uint32_t> perm(n);
  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t i = begin; i < end; ++i) perm[i] = keyIndex(keys[i]);
  });
  keys.release();

  parallelRanges(leaves.size(), workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t li = begin; li < end; ++li) {
      const Leaf& leaf = leaves[li];
      // Seeded per chunk so a reload produces byte-identical output.
      std::mt19937 rng(0x9E3779B9u ^ static_cast<std::uint32_t>(leaf.lo));
      std::shuffle(perm.begin() + static_cast<std::ptrdiff_t>(leaf.lo),
                   perm.begin() + static_cast<std::ptrdiff_t>(leaf.hi), rng);
    }
  });
  const double lodMs = lodTimer.elapsedMs();

  // --------------------------------------------------- 4. reorder arrays
  Stopwatch reorderTimer;
  gather(cloud.positions, perm, workers);
  if (!cloud.colors.empty()) gather(cloud.colors, perm, workers);
  if (!cloud.sourceIndex.empty()) gather(cloud.sourceIndex, perm, workers);
  for (ScalarColumn& column : cloud.columns) gatherColumn(column, perm, workers);
  const double reorderMs = reorderTimer.elapsedMs();

  // ------------------------------------------- 5. tight bounds + spacing
  //
  // The octree cell a chunk occupies is usually far larger than its points
  // actually span. Culling against the real extent rejects far more.
  Stopwatch boundsTimer;
  chunks.resize(leaves.size());
  parallelRanges(leaves.size(), workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t li = begin; li < end; ++li) {
      const Leaf& leaf = leaves[li];
      Aabbf box;
      for (std::size_t i = leaf.lo; i < leaf.hi; ++i) box.expand(cloud.positions[i]);
      Chunk& chunk = chunks[li];
      chunk.bounds = box;
      chunk.firstPoint = static_cast<std::uint32_t>(leaf.lo);
      chunk.pointCount = static_cast<std::uint32_t>(leaf.count());
      chunk.spacing = estimateSpacing(box, chunk.pointCount);
    }
  });

  // Clamp spacing against the cloud's typical value.
  //
  // Splat radius is derived from spacing, so a sparse chunk -- 21 stray
  // returns spread over 20 m -- would otherwise be drawn as 5 m discs. Its
  // spacing is not wrong; the inference is. Splat size stands for a point's
  // share of a SURFACE, and scattered noise is not a surface.
  //
  // The reference is the point-weighted median, so it reflects where the
  // data actually is rather than being dragged around by the many small,
  // sparse chunks at the edges of a scan.
  if (!chunks.empty()) {
    std::vector<std::pair<float, std::uint32_t>> bySpacing;
    bySpacing.reserve(chunks.size());
    std::uint64_t totalPoints = 0;
    for (const Chunk& c : chunks) {
      if (c.spacing > 0.0f && c.pointCount > 0) {
        bySpacing.emplace_back(c.spacing, c.pointCount);
        totalPoints += c.pointCount;
      }
    }
    std::sort(bySpacing.begin(), bySpacing.end());

    float medianSpacing = 0.0f;
    std::uint64_t seen = 0;
    for (const auto& [spacing, count] : bySpacing) {
      seen += count;
      if (seen * 2 >= totalPoints) { medianSpacing = spacing; break; }
    }
    if (medianSpacing > 0.0f) {
      const float lo = medianSpacing * 0.25f;
      const float hi = medianSpacing * 4.0f;
      for (Chunk& c : chunks) c.spacing = std::clamp(c.spacing, lo, hi);
    }
  }
  const double boundsMs = boundsTimer.elapsedMs();

  if (stats != nullptr) {
    stats->partitionMs = partitionMs;
    stats->lodShuffleMs = lodMs;
    stats->reorderMs = reorderMs;
    stats->boundsMs = boundsMs;
    stats->totalMs = totalTimer.elapsedMs();
    stats->chunkCount = chunks.size();
    std::uint32_t lo = 0xFFFFFFFFu, hi = 0;
    std::uint64_t sum = 0;
    for (const Chunk& c : chunks) {
      lo = std::min(lo, c.pointCount);
      hi = std::max(hi, c.pointCount);
      sum += c.pointCount;
    }
    stats->minChunkPoints = chunks.empty() ? 0 : lo;
    stats->maxChunkPoints = hi;
    stats->meanChunkPoints =
        chunks.empty() ? 0.0 : static_cast<double>(sum) / static_cast<double>(chunks.size());
  }

  return chunks;
}

}  // namespace punto
