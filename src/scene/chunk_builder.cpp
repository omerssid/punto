#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <numeric>
#include <random>

#include "core/large_buffer.hpp"
#include "core/log.hpp"
#include "core/parallel.hpp"
#include "scene/chunk.hpp"

namespace pcv {
namespace {

// Octant index: bit 0 is x >= centre, bit 1 is y, bit 2 is z.
inline int octantOf(const Vec3f& p, const Vec3f& centre) {
  return (p.x >= centre.x ? 1 : 0) | (p.y >= centre.y ? 2 : 0) | (p.z >= centre.z ? 4 : 0);
}

Aabbf childBox(const Aabbf& box, const Vec3f& centre, int octant) {
  Aabbf out;
  out.lo.x = (octant & 1) ? centre.x : box.lo.x;
  out.hi.x = (octant & 1) ? box.hi.x : centre.x;
  out.lo.y = (octant & 2) ? centre.y : box.lo.y;
  out.hi.y = (octant & 2) ? box.hi.y : centre.y;
  out.lo.z = (octant & 4) ? centre.z : box.lo.z;
  out.hi.z = (octant & 4) ? box.hi.z : centre.z;
  return out;
}

// The element the octree partitions.
//
// Carrying the position alongside the index -- rather than looking it up as
// positions[perm[i]] -- is the difference between sequential and scattered
// memory access. The indirection costs a cache miss per point per level,
// which measured at ~1.5 s on a 32M-point cloud; paying 16 bytes per point
// of scratch instead brings it down to a streaming copy.
struct PosIdx {
  Vec3f p;
  std::uint32_t index;
};
static_assert(sizeof(PosIdx) == 16);

struct Node {
  std::uint32_t lo = 0;
  std::uint32_t hi = 0;
  Aabbf box;
  int depth = 0;
  std::uint32_t count() const { return hi - lo; }
};

// Partitions items[lo, hi) into eight octant groups, serially.
// `offsets` receives the nine group boundaries, absolute into items.
void partitionSerial(PosIdx* items, PosIdx* scratch, std::uint32_t lo, std::uint32_t hi,
                     const Vec3f& centre, std::array<std::uint32_t, 9>& offsets) {
  std::array<std::uint32_t, 8> counts{};
  for (std::uint32_t i = lo; i < hi; ++i) ++counts[octantOf(items[i].p, centre)];

  offsets[0] = lo;
  for (int o = 0; o < 8; ++o) offsets[o + 1] = offsets[o] + counts[o];

  std::array<std::uint32_t, 8> cursor{};
  for (int o = 0; o < 8; ++o) cursor[o] = offsets[o];
  for (std::uint32_t i = lo; i < hi; ++i) {
    const PosIdx item = items[i];
    scratch[cursor[octantOf(item.p, centre)]++] = item;
  }
  std::memcpy(items + lo, scratch + lo, static_cast<std::size_t>(hi - lo) * sizeof(PosIdx));
}

// Same, but spread across workers. Used for the top levels, where a single
// node holds most of the cloud and there are too few nodes to parallelise
// over instead.
void partitionParallel(PosIdx* items, PosIdx* scratch, std::uint32_t lo, std::uint32_t hi,
                       const Vec3f& centre, unsigned workers,
                       std::array<std::uint32_t, 9>& offsets) {
  const std::size_t n = hi - lo;
  struct alignas(64) Counts {
    std::array<std::uint32_t, 8> c{};
  };
  std::vector<Counts> perWorker(workers);

  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
    auto& c = perWorker[w].c;
    for (std::size_t i = begin; i < end; ++i) ++c[octantOf(items[lo + i].p, centre)];
  });

  // Exclusive prefix sum over (octant, worker) so every worker knows exactly
  // where its share of each octant starts and can scatter without locking.
  std::array<std::uint32_t, 8> total{};
  for (const Counts& c : perWorker)
    for (int o = 0; o < 8; ++o) total[o] += c.c[o];

  offsets[0] = lo;
  for (int o = 0; o < 8; ++o) offsets[o + 1] = offsets[o] + total[o];

  std::vector<std::array<std::uint32_t, 8>> start(workers);
  std::array<std::uint32_t, 8> running{};
  for (int o = 0; o < 8; ++o) running[o] = offsets[o];
  for (unsigned w = 0; w < workers; ++w) {
    for (int o = 0; o < 8; ++o) {
      start[w][o] = running[o];
      running[o] += perWorker[w].c[o];
    }
  }

  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
    auto cursor = start[w];
    for (std::size_t i = begin; i < end; ++i) {
      const PosIdx item = items[lo + i];
      scratch[cursor[octantOf(item.p, centre)]++] = item;
    }
  });

  std::memcpy(items + lo, scratch + lo, n * sizeof(PosIdx));
}

// Reorders every per-point array so that `out[i] = in[perm[i]]`.
template <typename T>
void gather(std::vector<T>& values, const std::vector<std::uint32_t>& perm, unsigned workers) {
  if (values.empty()) return;
  const std::size_t n = values.size();
  // Gathered into uninitialised storage for the same reason as above, then
  // moved back into the vector so callers keep a plain std::vector.
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
// measure is area per point, not volume per point. Using the two largest box
// dimensions gives the area of the face the surface most likely spans.
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

  // ------------------------------------------------- 1. octree partition
  Stopwatch partitionTimer;
  if (!cloud.boundsLocal.valid()) cloud.recomputeBounds();

  // 16 bytes per point of working set, doubled for the scatter target.
  // Transient: both are released before the arrays are reordered.
  //
  // LargeBuffer rather than vector: no value-initialisation, and huge pages
  // so faulting in a gigabyte costs ~512 faults instead of ~262,000.
  LargeBuffer<PosIdx> items(n);
  LargeBuffer<PosIdx> scratch(n);
  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t i = begin; i < end; ++i)
      items[i] = PosIdx{cloud.positions[i], static_cast<std::uint32_t>(i)};
  });

  std::vector<Node> level{Node{0, static_cast<std::uint32_t>(n), cloud.boundsLocal, 0}};
  std::vector<Node> nextLevel;
  std::vector<Node> leaves;

  while (!level.empty()) {
    nextLevel.clear();

    // Nodes at this level cover disjoint ranges of perm, so they can be
    // partitioned concurrently. Early levels have too few nodes for that to
    // fill the machine, so those parallelise over points within each node
    // instead.
    const bool parallelOverNodes = level.size() >= workers;

    std::vector<std::array<std::uint32_t, 9>> splits(level.size());
    std::vector<char> isLeaf(level.size(), 0);

    auto processNode = [&](std::size_t ni, bool innerParallel) {
      const Node& node = level[ni];
      if (node.count() <= maxPoints || node.depth >= options.maxDepth) {
        isLeaf[ni] = 1;
        return;
      }
      const Vec3f centre = node.box.center();
      if (innerParallel)
        partitionParallel(items.get(), scratch.get(), node.lo, node.hi, centre, workers,
                          splits[ni]);
      else
        partitionSerial(items.get(), scratch.get(), node.lo, node.hi, centre, splits[ni]);
    };

    Stopwatch levelTimer;
    std::uint64_t levelPoints = 0;
    for (const Node& nd : level) levelPoints += nd.count();

    if (parallelOverNodes) {
      parallelRanges(level.size(), workers, [&](std::size_t begin, std::size_t end, unsigned) {
        for (std::size_t ni = begin; ni < end; ++ni) processNode(ni, false);
      });
    } else {
      for (std::size_t ni = 0; ni < level.size(); ++ni) processNode(ni, true);
    }

    logDebug("  level depth={} nodes={} points={} mode={} {:.1f} ms", level[0].depth,
             level.size(), levelPoints, parallelOverNodes ? "by-node" : "by-point",
             levelTimer.elapsedMs());

    for (std::size_t ni = 0; ni < level.size(); ++ni) {
      const Node& node = level[ni];
      if (isLeaf[ni]) {
        if (node.count() > 0) leaves.push_back(node);
        continue;
      }
      const Vec3f centre = node.box.center();
      for (int o = 0; o < 8; ++o) {
        const std::uint32_t lo = splits[ni][o];
        const std::uint32_t hi = splits[ni][o + 1];
        if (hi <= lo) continue;
        nextLevel.push_back(Node{lo, hi, childBox(node.box, centre, o), node.depth + 1});
      }
    }
    level.swap(nextLevel);
  }

  // Leaves come out in level order; sort so chunk index follows memory order,
  // which keeps the GPU's indirect draws reading the vertex buffer forward.
  std::sort(leaves.begin(), leaves.end(),
            [](const Node& a, const Node& b) { return a.lo < b.lo; });

  // Absorb undersized leaves into their neighbours.
  //
  // Splitting a node whose points cluster in one octant leaves siblings
  // holding a handful of points each, and a chunk of 1 point costs a whole
  // indirect draw command and a cull test to render nothing. Leaves are
  // sorted by offset and cover the range contiguously, and adjacent leaves
  // are octree siblings, so merging a run of them is both valid and
  // spatially sensible.
  const std::uint32_t minPoints = std::max<std::uint32_t>(1, maxPoints / 8);
  {
    std::vector<Node> merged;
    merged.reserve(leaves.size());
    for (const Node& leaf : leaves) {
      if (!merged.empty()) {
        Node& prev = merged.back();
        const bool prevTooSmall = prev.count() < minPoints;
        const bool leafTooSmall = leaf.count() < minPoints;
        const bool fits = prev.count() + leaf.count() <= maxPoints;
        if ((prevTooSmall || leafTooSmall) && fits && prev.hi == leaf.lo) {
          prev.hi = leaf.hi;
          prev.box.expand(leaf.box);
          prev.depth = std::min(prev.depth, leaf.depth);
          continue;
        }
      }
      merged.push_back(leaf);
    }
    leaves.swap(merged);
  }
  const double partitionMs = partitionTimer.elapsedMs();

  // ---------------------------------------------- 2. LOD order per chunk
  //
  // Shuffling the permutation (rather than the point data) means the whole
  // reorder is still a single gather pass. After this, the first N entries
  // of any chunk are a uniform random sample of that chunk, for every N --
  // which is what makes continuous LOD free.
  Stopwatch lodTimer;
  scratch.release();  // 512 MB on a 32M cloud; release it before the gather

  std::vector<std::uint32_t> perm(n);
  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t i = begin; i < end; ++i) perm[i] = items[i].index;
  });
  items.release();

  parallelRanges(leaves.size(), workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t li = begin; li < end; ++li) {
      const Node& node = leaves[li];
      // Seeded per chunk so a reload produces byte-identical output.
      std::mt19937 rng(0x9E3779B9u ^ static_cast<std::uint32_t>(node.lo));
      std::shuffle(perm.begin() + node.lo, perm.begin() + node.hi, rng);
    }
  });
  const double lodMs = lodTimer.elapsedMs();

  // --------------------------------------------------- 3. reorder arrays
  Stopwatch reorderTimer;
  gather(cloud.positions, perm, workers);
  if (!cloud.colors.empty()) gather(cloud.colors, perm, workers);
  if (!cloud.sourceIndex.empty()) gather(cloud.sourceIndex, perm, workers);
  for (ScalarColumn& column : cloud.columns) gatherColumn(column, perm, workers);
  const double reorderMs = reorderTimer.elapsedMs();

  // ------------------------------------------- 4. tight bounds + spacing
  //
  // The octree box a chunk inherited is usually far larger than the points
  // actually occupy. Culling against the real extent rejects far more.
  Stopwatch boundsTimer;
  chunks.resize(leaves.size());
  parallelRanges(leaves.size(), workers, [&](std::size_t begin, std::size_t end, unsigned) {
    for (std::size_t li = begin; li < end; ++li) {
      const Node& node = leaves[li];
      Aabbf box;
      for (std::uint32_t i = node.lo; i < node.hi; ++i) box.expand(cloud.positions[i]);
      Chunk& chunk = chunks[li];
      chunk.bounds = box;
      chunk.firstPoint = node.lo;
      chunk.pointCount = node.count();
      chunk.spacing = estimateSpacing(box, chunk.pointCount);
    }
  });
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

}  // namespace pcv
