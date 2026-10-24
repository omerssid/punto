#include "io/cloud_builder.hpp"

#include <algorithm>
#include <cmath>
#include <atomic>
#include <cstring>

#include "core/log.hpp"
#include "core/parallel.hpp"

namespace pcv {
namespace {

// Unaligned load. Record fields routinely sit at odd offsets, so this cannot
// just dereference a cast pointer. memcpy of a fixed small size compiles to a
// single move instruction.
template <typename T>
inline T loadUnaligned(const std::byte* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

inline double readAsDouble(const std::byte* p, FieldType t) {
  switch (t) {
    case FieldType::I8:  return static_cast<double>(loadUnaligned<std::int8_t>(p));
    case FieldType::I16: return static_cast<double>(loadUnaligned<std::int16_t>(p));
    case FieldType::I32: return static_cast<double>(loadUnaligned<std::int32_t>(p));
    case FieldType::I64: return static_cast<double>(loadUnaligned<std::int64_t>(p));
    case FieldType::U8:  return static_cast<double>(loadUnaligned<std::uint8_t>(p));
    case FieldType::U16: return static_cast<double>(loadUnaligned<std::uint16_t>(p));
    case FieldType::U32: return static_cast<double>(loadUnaligned<std::uint32_t>(p));
    case FieldType::U64: return static_cast<double>(loadUnaligned<std::uint64_t>(p));
    case FieldType::F32: return static_cast<double>(loadUnaligned<float>(p));
    case FieldType::F64: return loadUnaligned<double>(p);
  }
  return 0.0;
}

// Reinterprets the field's bytes as a 32-bit colour word. PCL stores packed
// RGB inside a float field, so the bits must be taken verbatim rather than
// numerically converted.
inline std::uint32_t readAsBits32(const std::byte* p, FieldType t) {
  if (fieldTypeSize(t) >= 4) return loadUnaligned<std::uint32_t>(p);
  if (fieldTypeSize(t) == 2) return loadUnaligned<std::uint16_t>(p);
  return loadUnaligned<std::uint8_t>(p);
}

inline std::uint32_t packRgba(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a) {
  // Little-endian byte order R,G,B,A so the array uploads directly as GL_RGBA8.
  return static_cast<std::uint32_t>(r) | (static_cast<std::uint32_t>(g) << 8) |
         (static_cast<std::uint32_t>(b) << 16) | (static_cast<std::uint32_t>(a) << 24);
}

inline std::uint8_t channelToByte(double v, bool normalizedFloat) {
  const double scaled = normalizedFloat ? v * 255.0 : v;
  return static_cast<std::uint8_t>(std::clamp(scaled, 0.0, 255.0));
}

std::string lower(std::string_view s) {
  std::string o(s);
  for (char& c : o) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return o;
}

FieldDest destForName(std::string_view rawName, FieldType type, bool& normalizedFloat) {
  const std::string n = lower(rawName);
  normalizedFloat = false;
  if (n == "x") return FieldDest::PosX;
  if (n == "y") return FieldDest::PosY;
  if (n == "z") return FieldDest::PosZ;
  if (n == "rgb") return FieldDest::PackedRgb;
  if (n == "rgba") return FieldDest::PackedRgba;
  // PLY writes colour as separate channels, sometimes as 0..1 floats.
  normalizedFloat = fieldTypeIsFloat(type);
  if (n == "r" || n == "red")   return FieldDest::ColorR;
  if (n == "g" || n == "green") return FieldDest::ColorG;
  if (n == "b" || n == "blue")  return FieldDest::ColorB;
  if (n == "a" || n == "alpha") return FieldDest::ColorA;
  normalizedFloat = false;
  return FieldDest::Column;
}

}  // namespace

bool positionFinite(const Vec3f& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

std::size_t layoutFields(std::vector<SourceField>& fields) {
  std::size_t offset = 0;
  for (SourceField& f : fields) {
    f.byteOffset = offset;
    offset += fieldTypeSize(f.type) * std::max<std::size_t>(1, f.count);
  }
  return offset;
}

DecodePlan buildDecodePlan(const std::vector<SourceField>& fields, PointCloud& cloud,
                           std::vector<std::string>& warnings) {
  DecodePlan plan;
  cloud.fields.clear();
  cloud.columns.clear();

  // First pass fixes the column count so `columns` never reallocates while
  // entries hold indices into it.
  std::size_t columnCount = 0;
  for (const SourceField& f : fields) {
    const std::size_t count = std::max<std::size_t>(1, f.count);
    for (std::size_t c = 0; c < count; ++c) {
      bool normFloat = false;
      // Only a count-1 field can be a position or colour channel; a
      // multi-count field is always a plain set of columns.
      const FieldDest dest =
          count == 1 ? destForName(f.name, f.type, normFloat) : FieldDest::Column;
      if (dest == FieldDest::Column) ++columnCount;
    }
  }
  cloud.columns.reserve(columnCount);

  for (const SourceField& f : fields) {
    const std::size_t count = std::max<std::size_t>(1, f.count);
    const std::size_t elemSize = fieldTypeSize(f.type);

    for (std::size_t c = 0; c < count; ++c) {
      DecodePlan::Entry entry;
      entry.type = f.type;
      entry.srcOffset = f.byteOffset + c * elemSize;

      bool normFloat = false;
      entry.dest = count == 1 ? destForName(f.name, f.type, normFloat) : FieldDest::Column;
      entry.colorIsNormalizedFloat = normFloat;

      // A field named x/y/z that appears twice would silently overwrite the
      // first; warn rather than produce a quietly wrong cloud.
      auto markPos = [&](int axis) {
        if (plan.hasPos[axis])
          warnings.push_back("duplicate position field '" + f.name + "' ignored");
        plan.hasPos[axis] = true;
      };

      std::string columnName = f.name;
      if (count > 1) columnName = f.name + "_" + std::to_string(c);

      switch (entry.dest) {
        case FieldDest::PosX: markPos(0); break;
        case FieldDest::PosY: markPos(1); break;
        case FieldDest::PosZ: markPos(2); break;
        case FieldDest::PackedRgb:
        case FieldDest::PackedRgba:
        case FieldDest::ColorR:
        case FieldDest::ColorG:
        case FieldDest::ColorB:
        case FieldDest::ColorA:
          plan.producesColor = true;
          break;
        case FieldDest::Column:
          entry.column = static_cast<int>(cloud.columns.size());
          cloud.columns.emplace_back();  // typed/sized later by allocateCloud
          break;
        case FieldDest::Skip:
          break;
      }

      FieldInfo info;
      info.name = columnName;
      info.type = f.type;
      info.role = inferFieldRole(columnName);
      info.columnIndex = entry.column;
      cloud.fields.push_back(std::move(info));

      plan.entries.push_back(entry);
    }
  }

  plan.tokensPerRecord = plan.entries.size();
  return plan;
}

void allocateCloud(PointCloud& cloud, const DecodePlan& plan, std::size_t n,
                   const LoadOptions& options) {
  cloud.positions.assign(n, Vec3f{0.0f, 0.0f, 0.0f});
  if (plan.producesColor) {
    // Opaque white, so a cloud with only some colour channels present still
    // renders sensibly.
    cloud.colors.assign(n, packRgba(255, 255, 255, 255));
  } else {
    cloud.colors.clear();
  }
  if (options.keepSourceIndex) {
    cloud.sourceIndex.resize(n);
  } else {
    cloud.sourceIndex.clear();
  }

  for (const DecodePlan::Entry& e : plan.entries) {
    if (e.dest == FieldDest::Column && e.column >= 0)
      cloud.columns[static_cast<std::size_t>(e.column)].reset(e.type, n);
  }
}

void setInterleavedStride(DecodePlan& plan, std::size_t recordStride) {
  plan.recordStride = recordStride;
  for (DecodePlan::Entry& e : plan.entries) e.srcStride = recordStride;
}

void setFieldBlockStrides(DecodePlan& plan, const std::vector<SourceField>& fields,
                          std::size_t pointCount) {
  // buildDecodePlan emits entries in declaration order, one per count
  // component, so fields and entries walk in lockstep.
  std::size_t blockStart = 0;
  std::size_t entryIndex = 0;
  for (const SourceField& f : fields) {
    const std::size_t count = std::max<std::size_t>(1, f.count);
    const std::size_t esz = fieldTypeSize(f.type);
    // A point occupies count*esz bytes inside this field's block, with its
    // components adjacent.
    const std::size_t slot = esz * count;
    for (std::size_t c = 0; c < count; ++c, ++entryIndex) {
      if (entryIndex >= plan.entries.size()) return;
      DecodePlan::Entry& e = plan.entries[entryIndex];
      e.srcOffset = blockStart + c * esz;
      e.srcStride = slot;
    }
    blockStart += slot * pointCount;
  }
  plan.recordStride = 0;  // meaningless for this layout
}

Vec3 chooseOrigin(const DecodePlan& plan, const std::byte* base, std::size_t pointCount) {
  if (pointCount == 0 || !plan.hasAllPositions()) return Vec3{0.0, 0.0, 0.0};

  // Sample across the whole file rather than taking the first point: a cloud
  // whose first point sits at one far corner would push the opposite corner
  // needlessly far from the origin.
  constexpr std::size_t kSamples = 4096;
  const std::size_t stride = std::max<std::size_t>(1, pointCount / kSamples);

  const DecodePlan::Entry* px = nullptr;
  const DecodePlan::Entry* py = nullptr;
  const DecodePlan::Entry* pz = nullptr;
  for (const DecodePlan::Entry& e : plan.entries) {
    if (e.dest == FieldDest::PosX && px == nullptr) px = &e;
    if (e.dest == FieldDest::PosY && py == nullptr) py = &e;
    if (e.dest == FieldDest::PosZ && pz == nullptr) pz = &e;
  }
  if (px == nullptr || py == nullptr || pz == nullptr) return Vec3{0.0, 0.0, 0.0};

  Aabb box;
  for (std::size_t i = 0; i < pointCount; i += stride) {
    const Vec3 p{readAsDouble(base + px->srcOffset + i * px->srcStride, px->type),
                 readAsDouble(base + py->srcOffset + i * py->srcStride, py->type),
                 readAsDouble(base + pz->srcOffset + i * pz->srcStride, pz->type)};
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    box.expand(p);
  }
  if (!box.valid()) return Vec3{0.0, 0.0, 0.0};

  // Round to whole metres so the origin reads cleanly in the UI and stays
  // identical across reloads.
  const Vec3 c = box.center();
  return Vec3{std::floor(c.x), std::floor(c.y), std::floor(c.z)};
}

std::uint64_t decodeRange(const DecodePlan& plan, const std::byte* base, std::size_t first,
                          std::size_t last, const Vec3& origin, bool keepSourceIndex,
                          PointCloud& cloud) {
  std::uint64_t invalid = 0;

  for (std::size_t i = first; i < last; ++i) {
    Vec3f pos{0.0f, 0.0f, 0.0f};
    std::uint8_t cr = 255, cg = 255, cb = 255, ca = 255;
    bool sawColor = false;

    for (const DecodePlan::Entry& e : plan.entries) {
      const std::byte* src = base + e.srcOffset + i * e.srcStride;
      switch (e.dest) {
        // Subtract the origin in double, then narrow. Narrowing first would
        // have already lost the precision the rebasing exists to preserve.
        case FieldDest::PosX: pos.x = static_cast<float>(readAsDouble(src, e.type) - origin.x); break;
        case FieldDest::PosY: pos.y = static_cast<float>(readAsDouble(src, e.type) - origin.y); break;
        case FieldDest::PosZ: pos.z = static_cast<float>(readAsDouble(src, e.type) - origin.z); break;

        case FieldDest::PackedRgb: {
          const std::uint32_t v = readAsBits32(src, e.type);
          cr = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
          cg = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
          cb = static_cast<std::uint8_t>(v & 0xFFu);
          ca = 255;
          sawColor = true;
          break;
        }
        case FieldDest::PackedRgba: {
          const std::uint32_t v = readAsBits32(src, e.type);
          ca = static_cast<std::uint8_t>((v >> 24) & 0xFFu);
          cr = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
          cg = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
          cb = static_cast<std::uint8_t>(v & 0xFFu);
          sawColor = true;
          break;
        }
        case FieldDest::ColorR:
          cr = channelToByte(readAsDouble(src, e.type), e.colorIsNormalizedFloat); sawColor = true; break;
        case FieldDest::ColorG:
          cg = channelToByte(readAsDouble(src, e.type), e.colorIsNormalizedFloat); sawColor = true; break;
        case FieldDest::ColorB:
          cb = channelToByte(readAsDouble(src, e.type), e.colorIsNormalizedFloat); sawColor = true; break;
        case FieldDest::ColorA:
          ca = channelToByte(readAsDouble(src, e.type), e.colorIsNormalizedFloat); sawColor = true; break;

        case FieldDest::Column: {
          // Source and destination share a type, so the bytes move verbatim.
          ScalarColumn& col = cloud.columns[static_cast<std::size_t>(e.column)];
          const std::size_t esz = col.elementSize();
          std::memcpy(col.data() + i * esz, src, esz);
          break;
        }
        case FieldDest::Skip:
          break;
      }
    }

    cloud.positions[i] = pos;
    if (sawColor && !cloud.colors.empty()) cloud.colors[i] = packRgba(cr, cg, cb, ca);
    if (keepSourceIndex && !cloud.sourceIndex.empty())
      cloud.sourceIndex[i] = static_cast<std::uint32_t>(i);
    if (!positionFinite(pos)) ++invalid;
  }

  return invalid;
}

bool decodeParallel(const DecodePlan& plan, const std::byte* base, std::size_t pointCount,
                    const Vec3& origin, const LoadOptions& options, std::stop_token stop,
                    PointCloud& cloud, std::uint64_t& invalidCount) {
  const unsigned workers = options.workers != 0 ? options.workers : defaultWorkerCount();
  std::vector<std::uint64_t> perWorkerInvalid(workers, 0);
  std::atomic<bool> cancelled{false};

  parallelRangesCancellable(
      pointCount, workers, std::move(stop),
      [&](std::size_t begin, std::size_t end, unsigned w, const std::stop_token& token) {
        // Check for cancellation per block rather than per point: the check
        // is an atomic load and would otherwise dominate the decode.
        constexpr std::size_t kCancelCheckBlock = 1u << 16;
        std::uint64_t invalid = 0;
        for (std::size_t s = begin; s < end; s += kCancelCheckBlock) {
          if (token.stop_requested()) {
            cancelled.store(true, std::memory_order_relaxed);
            break;
          }
          const std::size_t e = std::min(end, s + kCancelCheckBlock);
          invalid += decodeRange(plan, base, s, e, origin, options.keepSourceIndex, cloud);
        }
        perWorkerInvalid[w] = invalid;
      });

  if (cancelled.load(std::memory_order_relaxed)) return false;
  invalidCount = 0;
  for (std::uint64_t v : perWorkerInvalid) invalidCount += v;
  return true;
}

void compactInvalidPoints(PointCloud& cloud, std::uint64_t invalidCount) {
  if (invalidCount == 0) return;

  const std::size_t n = cloud.positions.size();
  const bool hasColors = !cloud.colors.empty();
  const bool hasIndex = !cloud.sourceIndex.empty();

  // Single sequential sweep that slides survivors down over the gaps.
  // Only runs when the file actually contained invalid points.
  std::size_t out = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (!positionFinite(cloud.positions[i])) continue;
    if (out != i) {
      cloud.positions[out] = cloud.positions[i];
      if (hasColors) cloud.colors[out] = cloud.colors[i];
      if (hasIndex) cloud.sourceIndex[out] = cloud.sourceIndex[i];
      for (ScalarColumn& col : cloud.columns) {
        const std::size_t esz = col.elementSize();
        std::memcpy(col.data() + out * esz, col.data() + i * esz, esz);
      }
    }
    ++out;
  }

  cloud.positions.resize(out);
  cloud.positions.shrink_to_fit();
  if (hasColors) { cloud.colors.resize(out); cloud.colors.shrink_to_fit(); }
  if (hasIndex) { cloud.sourceIndex.resize(out); cloud.sourceIndex.shrink_to_fit(); }
  for (ScalarColumn& col : cloud.columns) {
    ScalarColumn trimmed(col.type(), out);
    std::memcpy(trimmed.data(), col.data(), out * col.elementSize());
    col = std::move(trimmed);
  }

  cloud.invalidPointsDropped = invalidCount;
  // Dropping points destroys the row-major width x height grid.
  if (cloud.organized) {
    cloud.organized = false;
    cloud.width = static_cast<std::uint32_t>(out);
    cloud.height = 1;
  }
}

}  // namespace pcv
