#include "io/point_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/log.hpp"
#include "core/parallel.hpp"

namespace pcv {
namespace {

constexpr std::size_t kHistogramBins = 256;

// Stats for one position axis. Positions live in a Vec3f array rather than a
// ScalarColumn, so this walks them with a stride instead of going through
// ScalarColumn::visit.
FieldStats positionAxisStats(const std::vector<Vec3f>& positions, int axis, double originOffset) {
  FieldStats stats;
  const std::size_t n = positions.size();
  if (n == 0) return stats;

  struct alignas(64) Acc {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    double sum = 0.0;
    std::uint64_t finite = 0;
    std::uint64_t nonFinite = 0;
  };

  const unsigned workers = defaultWorkerCount();
  std::vector<Acc> accs(workers);
  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
    Acc& acc = accs[w];
    for (std::size_t i = begin; i < end; ++i) {
      const float raw = positions[i][axis];
      if (!std::isfinite(raw)) { ++acc.nonFinite; continue; }
      // Report in absolute world coordinates, which is what the user sees
      // in the file and in their other tooling.
      const double v = static_cast<double>(raw) + originOffset;
      acc.min = std::min(acc.min, v);
      acc.max = std::max(acc.max, v);
      acc.sum += v;
      ++acc.finite;
    }
  });

  Acc merged;
  for (const Acc& acc : accs) {
    merged.min = std::min(merged.min, acc.min);
    merged.max = std::max(merged.max, acc.max);
    merged.sum += acc.sum;
    merged.finite += acc.finite;
    merged.nonFinite += acc.nonFinite;
  }
  stats.finiteCount = merged.finite;
  stats.nonFiniteCount = merged.nonFinite;
  if (merged.finite == 0) return stats;
  stats.min = merged.min;
  stats.max = merged.max;
  stats.mean = merged.sum / static_cast<double>(merged.finite);

  const double range = stats.max - stats.min;
  const double binScale = range > 0.0 ? static_cast<double>(kHistogramBins) / range : 0.0;

  struct alignas(64) Acc2 {
    double sumSqDev = 0.0;
    std::vector<std::uint32_t> histogram = std::vector<std::uint32_t>(kHistogramBins, 0u);
  };
  std::vector<Acc2> accs2(workers);
  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
    Acc2& acc = accs2[w];
    for (std::size_t i = begin; i < end; ++i) {
      const float raw = positions[i][axis];
      if (!std::isfinite(raw)) continue;
      const double v = static_cast<double>(raw) + originOffset;
      const double d = v - stats.mean;
      acc.sumSqDev += d * d;
      const std::size_t bin =
          binScale > 0.0 ? std::min<std::size_t>(kHistogramBins - 1,
                                                 static_cast<std::size_t>((v - stats.min) * binScale))
                         : 0;
      ++acc.histogram[bin];
    }
  });

  double sumSqDev = 0.0;
  stats.histogram.assign(kHistogramBins, 0u);
  for (const Acc2& acc : accs2) {
    sumSqDev += acc.sumSqDev;
    for (std::size_t b = 0; b < kHistogramBins; ++b) stats.histogram[b] += acc.histogram[b];
  }
  stats.stddev = std::sqrt(sumSqDev / static_cast<double>(merged.finite));
  return stats;
}

// Statistics for one byte lane of the packed colour word.
FieldStats colorChannelStatsFor(const std::vector<std::uint32_t>& colors, int channel) {
  FieldStats stats;
  const std::size_t n = colors.size();
  if (n == 0) return stats;

  const unsigned shift = static_cast<unsigned>(channel) * 8u;
  struct alignas(64) Acc {
    std::uint64_t counts[256]{};
  };
  const unsigned workers = defaultWorkerCount();
  std::vector<Acc> accs(workers);
  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
    Acc& acc = accs[w];
    for (std::size_t i = begin; i < end; ++i) ++acc.counts[(colors[i] >> shift) & 0xFFu];
  });

  // A 256-value histogram is the exact distribution for a byte channel, so
  // every statistic follows from it without a second pass over the points.
  std::uint64_t counts[256] = {};
  for (const Acc& acc : accs)
    for (int v = 0; v < 256; ++v) counts[v] += acc.counts[v];

  stats.histogram.assign(256, 0u);
  double sum = 0.0;
  std::uint64_t total = 0;
  int lo = -1, hi = -1;
  for (int v = 0; v < 256; ++v) {
    const std::uint64_t c = counts[v];
    stats.histogram[static_cast<std::size_t>(v)] = static_cast<std::uint32_t>(std::min<std::uint64_t>(c, 0xFFFFFFFFull));
    if (c == 0) continue;
    if (lo < 0) lo = v;
    hi = v;
    sum += static_cast<double>(v) * static_cast<double>(c);
    total += c;
  }
  if (total == 0) return stats;

  stats.finiteCount = total;
  stats.min = lo;
  stats.max = hi;
  stats.mean = sum / static_cast<double>(total);
  double sumSq = 0.0;
  for (int v = 0; v < 256; ++v) {
    if (counts[v] == 0) continue;
    const double d = static_cast<double>(v) - stats.mean;
    sumSq += d * d * static_cast<double>(counts[v]);
  }
  stats.stddev = std::sqrt(sumSq / static_cast<double>(total));
  return stats;
}

// Fraction of points whose three channels are equal. Sampled, because the
// answer is a yes/no and a few thousand points settle it.
double greyscaleFraction(const std::vector<std::uint32_t>& colors) {
  const std::size_t n = colors.size();
  if (n == 0) return 0.0;
  constexpr std::size_t kSamples = 8192;
  const std::size_t step = std::max<std::size_t>(1, n / kSamples);

  std::size_t tested = 0;
  std::size_t equal = 0;
  for (std::size_t i = 0; i < n; i += step) {
    const std::uint32_t c = colors[i];
    const std::uint32_t r = c & 0xFFu;
    const std::uint32_t g = (c >> 8) & 0xFFu;
    const std::uint32_t b = (c >> 16) & 0xFFu;
    if (r == g && g == b) ++equal;
    ++tested;
  }
  return tested > 0 ? static_cast<double>(equal) / static_cast<double>(tested) : 0.0;
}

}  // namespace

const FieldInfo* PointCloud::findField(std::string_view name) const {
  for (const FieldInfo& f : fields)
    if (f.name == name) return &f;
  return nullptr;
}

std::vector<int> PointCloud::colorableFieldIndices() const {
  std::vector<int> out;
  out.reserve(fields.size());
  for (int i = 0; i < static_cast<int>(fields.size()); ++i) {
    const FieldInfo& f = fields[i];
    // x/y/z are colourable (height ramps) even though they have no column;
    // packed rgb is handled by its own render mode, not the scalar ramp.
    const bool isPositionAxis = f.role == FieldRole::Position;
    if (isPositionAxis || f.columnIndex >= 0) out.push_back(i);
  }
  return out;
}

std::uint64_t PointCloud::memoryBytes() const {
  std::uint64_t total = positions.capacity() * sizeof(Vec3f);
  total += colors.capacity() * sizeof(std::uint32_t);
  total += sourceIndex.capacity() * sizeof(std::uint32_t);
  for (const ScalarColumn& c : columns) total += c.byteSize();
  return total;
}

void PointCloud::recomputeBounds() {
  boundsLocal = Aabbf{};
  const std::size_t n = positions.size();
  if (n == 0) {
    boundsWorld = Aabb{};
    return;
  }

  struct alignas(64) Acc {
    Aabbf box;
  };
  const unsigned workers = defaultWorkerCount();
  std::vector<Acc> accs(workers);
  parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
    Aabbf box;
    for (std::size_t i = begin; i < end; ++i) {
      const Vec3f& p = positions[i];
      // NaN points are common in organised clouds and must not poison the box.
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      box.expand(p);
    }
    accs[w].box = box;
  });
  for (const Acc& acc : accs)
    if (acc.box.valid()) boundsLocal.expand(acc.box);

  if (boundsLocal.valid()) {
    boundsWorld.lo = origin + Vec3(boundsLocal.lo);
    boundsWorld.hi = origin + Vec3(boundsLocal.hi);
  } else {
    boundsWorld = Aabb{};
  }
}

void PointCloud::computeAllStats() {
  colorStatsValid = false;
  if (hasColors()) {
    for (int ch = 0; ch < 4; ++ch) colorChannelStats[static_cast<std::size_t>(ch)] = colorChannelStatsFor(colors, ch);
    colorStatsValid = true;
    colorGreyscaleFraction = greyscaleFraction(colors);
    colorIsGreyscale = colorGreyscaleFraction > 0.99;
  }

  for (FieldInfo& f : fields) {
    if (f.role == FieldRole::Position) {
      const int axis = f.name == "x" ? 0 : (f.name == "y" ? 1 : 2);
      f.stats = positionAxisStats(positions, axis, origin[axis]);
    } else if (f.columnIndex >= 0 && f.columnIndex < static_cast<int>(columns.size())) {
      f.stats = computeFieldStats(columns[f.columnIndex], f.role);
    }
  }
}

void PointCloud::clear() {
  origin = Vec3{0.0, 0.0, 0.0};
  positions.clear();
  positions.shrink_to_fit();
  colors.clear();
  colors.shrink_to_fit();
  sourceIndex.clear();
  sourceIndex.shrink_to_fit();
  columns.clear();
  fields.clear();
  boundsLocal = Aabbf{};
  boundsWorld = Aabb{};
  sourcePath.clear();
  formatName.clear();
  sourceBytes = 0;
  timing = LoadTiming{};
  organized = false;
  colorStatsValid = false;
  colorChannelStats = {};
  colorIsGreyscale = false;
  colorGreyscaleFraction = 0.0;
  invalidPointsDropped = 0;
  width = height = 0;
}

}  // namespace pcv
