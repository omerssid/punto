#include "io/field.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string_view>
#include <type_traits>

#include "core/parallel.hpp"

namespace pcv {
namespace {

// A column with at most this many distinct values is treated as categorical
// (ring indices, semantic labels) and gets a value list in the UI.
constexpr std::size_t kMaxDistinctTracked = 64;
constexpr std::size_t kHistogramBins = 256;

std::string toLowerAscii(std::string_view s) {
  std::string out(s);
  for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return out;
}

template <typename T>
bool isFiniteValue(T v) {
  if constexpr (std::is_floating_point_v<T>) return std::isfinite(v);
  else return true;
}

}  // namespace

std::size_t fieldTypeSize(FieldType t) {
  switch (t) {
    case FieldType::I8:  case FieldType::U8:  return 1;
    case FieldType::I16: case FieldType::U16: return 2;
    case FieldType::I32: case FieldType::U32: case FieldType::F32: return 4;
    case FieldType::I64: case FieldType::U64: case FieldType::F64: return 8;
  }
  return 4;
}

const char* fieldTypeName(FieldType t) {
  switch (t) {
    case FieldType::I8:  return "int8";
    case FieldType::I16: return "int16";
    case FieldType::I32: return "int32";
    case FieldType::I64: return "int64";
    case FieldType::U8:  return "uint8";
    case FieldType::U16: return "uint16";
    case FieldType::U32: return "uint32";
    case FieldType::U64: return "uint64";
    case FieldType::F32: return "float32";
    case FieldType::F64: return "float64";
  }
  return "?";
}

bool fieldTypeIsFloat(FieldType t) { return t == FieldType::F32 || t == FieldType::F64; }

bool fieldTypeIsSigned(FieldType t) {
  switch (t) {
    case FieldType::I8: case FieldType::I16: case FieldType::I32: case FieldType::I64:
    case FieldType::F32: case FieldType::F64:
      return true;
    default:
      return false;
  }
}

bool fieldTypeFromPcd(char typeChar, std::size_t size, FieldType& out) {
  switch (std::toupper(static_cast<unsigned char>(typeChar))) {
    case 'I':
      switch (size) {
        case 1: out = FieldType::I8;  return true;
        case 2: out = FieldType::I16; return true;
        case 4: out = FieldType::I32; return true;
        case 8: out = FieldType::I64; return true;
      }
      return false;
    case 'U':
      switch (size) {
        case 1: out = FieldType::U8;  return true;
        case 2: out = FieldType::U16; return true;
        case 4: out = FieldType::U32; return true;
        case 8: out = FieldType::U64; return true;
      }
      return false;
    case 'F':
      switch (size) {
        case 4: out = FieldType::F32; return true;
        case 8: out = FieldType::F64; return true;
      }
      return false;
    default:
      return false;
  }
}

FieldRole inferFieldRole(std::string_view name) {
  const std::string n = toLowerAscii(name);
  if (n == "x" || n == "y" || n == "z") return FieldRole::Position;
  if (n == "intensity" || n == "i" || n == "reflectance" || n == "intensities") return FieldRole::Intensity;
  if (n == "rgb" || n == "rgba" || n == "r" || n == "g" || n == "b" || n == "a" ||
      n == "red" || n == "green" || n == "blue" || n == "alpha")
    return FieldRole::Color;
  if (n == "normal_x" || n == "normal_y" || n == "normal_z" || n == "nx" || n == "ny" || n == "nz")
    return FieldRole::Normal;
  if (n == "label" || n == "class" || n == "classification" || n == "semantic" || n == "instance")
    return FieldRole::Label;
  if (n == "ring" || n == "line" || n == "laser_id" || n == "scan_id" || n == "channel")
    return FieldRole::Ring;
  if (n == "t" || n == "time" || n == "timestamp" || n == "time_offset" || n == "gps_time" ||
      n == "stamp" || n == "time_stamp")
    return FieldRole::Time;
  return FieldRole::Other;
}

const char* fieldRoleName(FieldRole r) {
  switch (r) {
    case FieldRole::Position:  return "position";
    case FieldRole::Intensity: return "intensity";
    case FieldRole::Color:     return "color";
    case FieldRole::Normal:    return "normal";
    case FieldRole::Label:     return "label";
    case FieldRole::Ring:      return "ring";
    case FieldRole::Time:      return "time";
    case FieldRole::Other:     return "other";
  }
  return "other";
}

void ScalarColumn::reset(FieldType type, std::size_t count) {
  type_ = type;
  count_ = count;
  bytes_.assign(count * fieldTypeSize(type), std::byte{0});
}

double ScalarColumn::at(std::size_t i) const {
  if (i >= count_) return 0.0;
  switch (type_) {
    case FieldType::I8:  return static_cast<double>(typedData<std::int8_t>()[i]);
    case FieldType::I16: return static_cast<double>(typedData<std::int16_t>()[i]);
    case FieldType::I32: return static_cast<double>(typedData<std::int32_t>()[i]);
    case FieldType::I64: return static_cast<double>(typedData<std::int64_t>()[i]);
    case FieldType::U8:  return static_cast<double>(typedData<std::uint8_t>()[i]);
    case FieldType::U16: return static_cast<double>(typedData<std::uint16_t>()[i]);
    case FieldType::U32: return static_cast<double>(typedData<std::uint32_t>()[i]);
    case FieldType::U64: return static_cast<double>(typedData<std::uint64_t>()[i]);
    case FieldType::F32: return static_cast<double>(typedData<float>()[i]);
    case FieldType::F64: return typedData<double>()[i];
  }
  return 0.0;
}

namespace {

// Per-worker partials for the first pass, padded to a cache line so adjacent
// workers never share one and ping-pong it between cores.
struct alignas(64) Pass1 {
  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::lowest();
  double sum = 0.0;
  std::uint64_t finite = 0;
  std::uint64_t nonFinite = 0;
  // Sorted, deduplicated, and abandoned once it would exceed the cap.
  std::vector<double> distinct;
  bool distinctOverflow = false;

  void noteDistinct(double v) {
    if (distinctOverflow) return;
    const auto it = std::lower_bound(distinct.begin(), distinct.end(), v);
    if (it != distinct.end() && *it == v) return;
    if (distinct.size() >= kMaxDistinctTracked) {
      distinctOverflow = true;
      distinct.clear();
      return;
    }
    distinct.insert(it, v);
  }
};

struct alignas(64) Pass2 {
  double sumSqDev = 0.0;
  std::vector<std::uint32_t> histogram = std::vector<std::uint32_t>(kHistogramBins, 0u);
};

}  // namespace

FieldStats computeFieldStats(const ScalarColumn& column, FieldRole role) {
  FieldStats stats;
  const std::size_t n = column.count();
  if (n == 0) return stats;

  // Track distinct values only where a small set would be meaningful.
  const bool trackDistinct =
      (role == FieldRole::Ring || role == FieldRole::Label) && !fieldTypeIsFloat(column.type());

  const unsigned workers = defaultWorkerCount();
  std::vector<Pass1> p1(workers);

  // Pass 1: range, sum, finite counts. Split so each worker walks a
  // contiguous block, which keeps the prefetcher happy.
  column.visit([&](auto span) {
    using T = typename decltype(span)::value_type;
    parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
      Pass1& acc = p1[w];
      for (std::size_t i = begin; i < end; ++i) {
        const T raw = span[i];
        if (!isFiniteValue(raw)) { ++acc.nonFinite; continue; }
        const double v = static_cast<double>(raw);
        acc.min = std::min(acc.min, v);
        acc.max = std::max(acc.max, v);
        acc.sum += v;
        ++acc.finite;
        if (trackDistinct) acc.noteDistinct(v);
      }
    });
  });

  Pass1 merged;
  bool anyOverflow = false;
  for (const Pass1& acc : p1) {
    merged.min = std::min(merged.min, acc.min);
    merged.max = std::max(merged.max, acc.max);
    merged.sum += acc.sum;
    merged.finite += acc.finite;
    merged.nonFinite += acc.nonFinite;
    anyOverflow = anyOverflow || acc.distinctOverflow;
  }

  stats.finiteCount = merged.finite;
  stats.nonFiniteCount = merged.nonFinite;
  if (merged.finite == 0) return stats;

  stats.min = merged.min;
  stats.max = merged.max;
  stats.mean = merged.sum / static_cast<double>(merged.finite);

  if (trackDistinct && !anyOverflow) {
    for (const Pass1& acc : p1)
      for (double v : acc.distinct) merged.noteDistinct(v);
    if (!merged.distinctOverflow) {
      stats.discrete = true;
      stats.distinctValues = std::move(merged.distinct);
    }
  }

  // Pass 2: variance about the now-known mean, plus the histogram. Deferring
  // variance to a second pass avoids the catastrophic cancellation that a
  // naive sum-of-squares accumulator suffers over tens of millions of values.
  const double range = stats.max - stats.min;
  const double binScale = range > 0.0 ? static_cast<double>(kHistogramBins) / range : 0.0;
  std::vector<Pass2> p2(workers);

  column.visit([&](auto span) {
    using T = typename decltype(span)::value_type;
    parallelRanges(n, workers, [&](std::size_t begin, std::size_t end, unsigned w) {
      Pass2& acc = p2[w];
      for (std::size_t i = begin; i < end; ++i) {
        const T raw = span[i];
        if (!isFiniteValue(raw)) continue;
        const double v = static_cast<double>(raw);
        const double d = v - stats.mean;
        acc.sumSqDev += d * d;
        const std::size_t bin =
            binScale > 0.0
                ? std::min<std::size_t>(kHistogramBins - 1,
                                        static_cast<std::size_t>((v - stats.min) * binScale))
                : 0;
        ++acc.histogram[bin];
      }
    });
  });

  double sumSqDev = 0.0;
  stats.histogram.assign(kHistogramBins, 0u);
  for (const Pass2& acc : p2) {
    sumSqDev += acc.sumSqDev;
    for (std::size_t b = 0; b < kHistogramBins; ++b) stats.histogram[b] += acc.histogram[b];
  }
  stats.stddev = std::sqrt(sumSqDev / static_cast<double>(merged.finite));
  return stats;
}

}  // namespace pcv
