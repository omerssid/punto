// LAS reader. LAZ is the same container with compressed point records and
// needs laszip, which is gated behind PUNTO_HAVE_LASZIP.
//
// Unlike PCD and PLY, LAS has a fixed schema: a point data record format
// number selects one of eleven layouts. Coordinates are stored as 32-bit
// integers with a per-file scale and offset, which is why the shared decode
// path carries an affine transform.
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <format>
#include <cstring>
#include <string>
#include <vector>

#include "core/large_buffer.hpp"
#include "core/log.hpp"
#include "core/parallel.hpp"
#include "io/cloud_builder.hpp"
#include "io/cloud_info.hpp"
#include "io/laz_decode.hpp"
#include "io/mapped_file.hpp"

namespace punto {
namespace {

struct LasHeader {
  std::uint8_t versionMajor = 1;
  std::uint8_t versionMinor = 2;
  std::uint16_t headerSize = 227;
  std::uint32_t pointDataOffset = 0;
  std::uint8_t pointFormat = 0;
  std::uint16_t pointRecordLength = 20;
  std::uint64_t pointCount = 0;
  double scale[3] = {0.01, 0.01, 0.01};
  double offset[3] = {0.0, 0.0, 0.0};
  double minBound[3] = {0.0, 0.0, 0.0};
  double maxBound[3] = {0.0, 0.0, 0.0};
  bool compressed = false;
};

template <typename T>
T read(const std::byte* base, std::size_t offset) {
  T v;
  std::memcpy(&v, base + offset, sizeof(T));
  return v;
}

bool parseHeader(const MappedFile& file, LasHeader& header, std::string& error) {
  if (file.size() < 227) {
    error = "LAS: file is shorter than a header";
    return false;
  }
  const auto* base = reinterpret_cast<const std::byte*>(file.chars());
  if (std::memcmp(file.chars(), "LASF", 4) != 0) {
    error = "LAS: missing LASF signature";
    return false;
  }

  header.versionMajor = read<std::uint8_t>(base, 24);
  header.versionMinor = read<std::uint8_t>(base, 25);
  header.headerSize = read<std::uint16_t>(base, 94);
  header.pointDataOffset = read<std::uint32_t>(base, 96);
  const std::uint8_t formatByte = read<std::uint8_t>(base, 104);
  header.pointRecordLength = read<std::uint16_t>(base, 105);

  // The high bits of the format byte flag LAZ compression; laszip sets
  // bit 7 (and historically bit 6) and leaves the format in the low bits.
  header.compressed = (formatByte & 0x80u) != 0 || (formatByte & 0x40u) != 0;
  header.pointFormat = static_cast<std::uint8_t>(formatByte & 0x3Fu);

  const std::uint32_t legacyCount = read<std::uint32_t>(base, 107);
  for (int i = 0; i < 3; ++i) {
    header.scale[i] = read<double>(base, 131 + static_cast<std::size_t>(i) * 8);
    header.offset[i] = read<double>(base, 155 + static_cast<std::size_t>(i) * 8);
  }
  header.maxBound[0] = read<double>(base, 179);
  header.minBound[0] = read<double>(base, 187);
  header.maxBound[1] = read<double>(base, 195);
  header.minBound[1] = read<double>(base, 203);
  header.maxBound[2] = read<double>(base, 211);
  header.minBound[2] = read<double>(base, 219);

  header.pointCount = legacyCount;
  // 1.4 moved the count to a 64-bit field, because the legacy one caps at
  // 4.29 billion points.
  if (header.versionMajor == 1 && header.versionMinor >= 4 && file.size() >= 255) {
    const std::uint64_t count64 = read<std::uint64_t>(base, 247);
    if (count64 > 0) header.pointCount = count64;
  }

  if (header.pointRecordLength == 0) {
    error = "LAS: point record length is zero";
    return false;
  }
  if (header.pointCount == 0) {
    error = "LAS: declares zero point records";
    return false;
  }
  if (header.pointDataOffset >= file.size()) {
    error = "LAS: point data offset lies past the end of the file";
    return false;
  }
  return true;
}

// Field layout for a point data record format. Offsets are within the
// record; a format's record may be longer than the fields listed here
// (waveform packets), which the stride accounts for.
struct LasLayout {
  std::vector<SourceField> fields;
  bool hasColor = false;
  int classificationIndex = -1;  // into fields, for the flag-bit mask
  bool legacyClassification = false;
};

bool layoutForFormat(std::uint8_t format, LasLayout& layout, std::string& error) {
  auto add = [&](const char* name, FieldType type) {
    layout.fields.push_back(SourceField{name, type, 1, 0});
  };

  // Formats 0-5 share a 20-byte core; 6-10 share a 30-byte core.
  const bool legacy = format <= 5;
  if (format > 10) {
    error = "LAS: unknown point data record format " + std::to_string(format);
    return false;
  }

  add("x", FieldType::I32);
  add("y", FieldType::I32);
  add("z", FieldType::I32);
  add("intensity", FieldType::U16);

  if (legacy) {
    add("return_info", FieldType::U8);   // return number, count, scan direction, edge
    add("classification", FieldType::U8);
    layout.classificationIndex = static_cast<int>(layout.fields.size()) - 1;
    layout.legacyClassification = true;
    add("scan_angle", FieldType::I8);
    add("user_data", FieldType::U8);
    add("point_source_id", FieldType::U16);
    if (format == 1 || format == 3 || format == 4 || format == 5) add("gps_time", FieldType::F64);
    if (format == 2 || format == 3 || format == 5) {
      add("red", FieldType::U16);
      add("green", FieldType::U16);
      add("blue", FieldType::U16);
      layout.hasColor = true;
    }
  } else {
    add("return_info", FieldType::U8);
    add("class_flags", FieldType::U8);
    add("classification", FieldType::U8);
    layout.classificationIndex = static_cast<int>(layout.fields.size()) - 1;
    add("user_data", FieldType::U8);
    add("scan_angle", FieldType::I16);
    add("point_source_id", FieldType::U16);
    add("gps_time", FieldType::F64);
    if (format >= 7) {
      add("red", FieldType::U16);
      add("green", FieldType::U16);
      add("blue", FieldType::U16);
      layout.hasColor = true;
    }
    if (format == 8 || format == 10) add("nir", FieldType::U16);
  }
  return true;
}

// LAS stores colour in 16-bit channels, but a great many files written from
// 8-bit imagery leave the values in 0..255 rather than scaling up. Sampling
// decides which, since guessing wrong makes a cloud either black or
// saturated.
bool colorLooksEightBit(const std::byte* base, std::size_t stride, std::size_t count,
                        std::size_t redOffset) {
  constexpr std::size_t kSamples = 4096;
  const std::size_t step = std::max<std::size_t>(1, count / kSamples);
  std::uint16_t peak = 0;
  for (std::size_t i = 0; i < count; i += step) {
    const std::byte* record = base + i * stride;
    for (int c = 0; c < 3; ++c) {
      std::uint16_t v = 0;
      std::memcpy(&v, record + redOffset + static_cast<std::size_t>(c) * 2, 2);
      peak = std::max(peak, v);
    }
  }
  return peak <= 255;
}

}  // namespace

LoadResult loadLas(const std::string& path, const LoadOptions& options, PointCloud& out,
                   std::stop_token stop) {
  ScopedTimer totalTimer("loadLas");
  LoadResult result;
  out.clear();

  Stopwatch mapTimer;
  MappedFile file;
  std::string error;
  if (!file.open(path, error)) return LoadResult::failure(std::move(error));
  out.timing.mapMs = mapTimer.elapsedMs();

  Stopwatch headerTimer;
  LasHeader header;
  if (!parseHeader(file, header, error)) return LoadResult::failure(std::move(error));
  out.timing.headerMs = headerTimer.elapsedMs();
  // Recorded before the mapping may be released for the LAZ path.
  const std::uint64_t fileBytes = file.size();

  if (header.compressed && !lazSupported()) {
    return LoadResult::failure(
        "LAZ files are compressed and this build has no laszip. Install "
        "liblaszip-dev and re-run cmake.");
  }

  LasLayout layout;
  if (!layoutForFormat(header.pointFormat, layout, error))
    return LoadResult::failure(std::move(error));

  // layoutFields computes the natural packing; LAS records match it for the
  // fields above, and any extra bytes are covered by the declared stride.
  const std::size_t naturalStride = layoutFields(layout.fields);
  std::size_t pointCount = static_cast<std::size_t>(header.pointCount);

  // Decompressed records are written in the natural packing, with none of
  // the vendor padding an uncompressed file may carry.
  std::size_t stride = header.compressed ? naturalStride : header.pointRecordLength;

  if (!header.compressed) {
    if (stride < naturalStride) {
      return LoadResult::failure("LAS: record length " + std::to_string(stride) +
                                 " is shorter than format " +
                                 std::to_string(header.pointFormat) + " requires (" +
                                 std::to_string(naturalStride) + ")");
    }
    if (stride > naturalStride)
      result.warnings.push_back("LAS: records carry " + std::to_string(stride - naturalStride) +
                                " extra bytes (waveform or vendor data), ignored");

    const std::size_t available = file.size() - header.pointDataOffset;
    if (available < stride * pointCount) {
      result.warnings.push_back("LAS: point block is truncated; reading " +
                                std::to_string(available / stride) + " of " +
                                std::to_string(pointCount) + " points");
      pointCount = available / stride;
    }
  }
  if (pointCount == 0) return LoadResult::failure("LAS: no complete point records");

  DecodePlan plan = buildDecodePlan(layout.fields, out, result.warnings, options);
  if (!plan.hasAllPositions()) return LoadResult::failure("LAS: internal layout error");
  setInterleavedStride(plan, stride);

  // Coordinates are integers; the file's scale and offset turn them into
  // metres. Applying it during the decode avoids a second pass.
  int axis = 0;
  for (DecodePlan::Entry& e : plan.entries) {
    if (e.dest == FieldDest::PosX) axis = 0;
    else if (e.dest == FieldDest::PosY) axis = 1;
    else if (e.dest == FieldDest::PosZ) axis = 2;
    else continue;
    e.scale = header.scale[axis];
    e.offset = header.offset[axis];
  }

  // Decompressed LAZ records live here; for LAS the mapping is read in
  // place and this stays empty.
  LargeBuffer<std::byte> decompressed;
  const std::byte* base = nullptr;

  Stopwatch decodeTimer;
  if (header.compressed) {
    // The mapping was only needed for the header.
    file.close();
    std::string lazError;
    if (!decodeLazPoints(path, layout.fields, stride, header.pointFormat, pointCount,
                         options.workers, stop, decompressed, result.warnings, lazError)) {
      if (lazError == "load cancelled") {
        result.cancelled = true;
        result.error = lazError;
        return result;
      }
      return LoadResult::failure(std::move(lazError));
    }
    base = decompressed.get();
  } else {
    base = reinterpret_cast<const std::byte*>(file.chars() + header.pointDataOffset);
    file.adviseSequential(header.pointDataOffset, pointCount * stride);
  }

  if (layout.hasColor) {
    std::size_t redOffset = 0;
    for (const SourceField& f : layout.fields)
      if (f.name == "red") redOffset = f.byteOffset;
    const bool eightBit = colorLooksEightBit(base, stride, pointCount, redOffset);
    if (!eightBit) {
      // Scale 16-bit channels down to the 8 bits the colour buffer holds.
      for (DecodePlan::Entry& e : plan.entries) {
        if (e.dest == FieldDest::ColorR || e.dest == FieldDest::ColorG ||
            e.dest == FieldDest::ColorB)
          e.scale = 255.0 / 65535.0;
      }
    } else {
      result.warnings.push_back(
          "LAS: colour channels are 16-bit but hold values of 255 or less; "
          "treating them as 8-bit");
    }
  }

  out.sourcePath = path;
  out.sourceBytes = fileBytes;
  out.formatName = std::format("{} {}.{} (format {})", header.compressed ? "LAZ" : "LAS",
                               header.versionMajor, header.versionMinor, header.pointFormat);
  out.width = static_cast<std::uint32_t>(pointCount);
  out.height = 1;

  out.origin = chooseOrigin(plan, base, pointCount);
  const std::size_t keptCount = applyPointLimit(plan, pointCount, options);
  allocateCloud(out, plan, keptCount, options);

  std::uint64_t invalid = 0;
  if (!decodeParallel(plan, base, keptCount, out.origin, options, stop, out, invalid)) {
    out.clear();
    result.cancelled = true;
    result.error = "load cancelled";
    return result;
  }
  out.timing.decodeMs = decodeTimer.elapsedMs();

  // In formats 0-5 the classification byte packs three flags into its top
  // bits, so a class of 6 can arrive as 134. Mask them off, because the
  // class number is what anyone colouring by classification wants.
  if (layout.legacyClassification && layout.classificationIndex >= 0) {
    const FieldInfo* info = out.findField("classification");
    if (info != nullptr && info->columnIndex >= 0) {
      ScalarColumn& column = out.columns[static_cast<std::size_t>(info->columnIndex)];
      auto* values = column.typedData<std::uint8_t>();
      std::atomic<std::uint64_t> flagged{0};
      parallelRanges(column.count(), [&](std::size_t begin, std::size_t end, unsigned) {
        std::uint64_t local = 0;
        for (std::size_t i = begin; i < end; ++i) {
          if ((values[i] & 0xE0u) != 0) ++local;
          values[i] &= 0x1Fu;
        }
        flagged.fetch_add(local, std::memory_order_relaxed);
      });
      const std::uint64_t count = flagged.load();
      if (count > 0)
        result.warnings.push_back(
            std::to_string(count) +
            " points carry synthetic/key-point/withheld flags; classification "
            "has been masked to the class number");
    }
  }

  if (options.dropInvalid) compactInvalidPoints(out, invalid);
  out.recomputeBounds();

  // The header declares the extent too; disagreeing with it usually means
  // the file was edited without updating the header.
  if (out.boundsWorld.valid()) {
    const double tolerance = 1e-3;
    for (int i = 0; i < 3; ++i) {
      if (std::abs(out.boundsWorld.lo[i] - header.minBound[i]) > tolerance ||
          std::abs(out.boundsWorld.hi[i] - header.maxBound[i]) > tolerance) {
        result.warnings.push_back(
            "LAS: the header's declared extent does not match the points");
        break;
      }
    }
  }

  Stopwatch statsTimer;
  out.computeAllStats();
  out.timing.statsMs = statsTimer.elapsedMs();

  out.timing.totalMs = totalTimer.elapsedMs();
  result.ok = true;
  return result;
}


bool probeLasHeader(const std::string& path, CloudInfo& info) {
  MappedFile file;
  std::string error;
  if (!file.open(path, error)) {
    info.error = std::move(error);
    return false;
  }

  LasHeader header;
  if (!parseHeader(file, header, info.error)) return false;

  LasLayout layout;
  if (!layoutForFormat(header.pointFormat, layout, info.error)) return false;

  const std::size_t naturalStride = layoutFields(layout.fields);
  info.pointCount = header.pointCount;
  info.fields = std::move(layout.fields);
  info.hasColor = layout.hasColor;
  info.recordStride = header.compressed ? naturalStride : header.pointRecordLength;
  info.formatName = std::format("{} 1.{} (format {})", header.compressed ? "LAZ" : "LAS",
                                header.versionMinor, header.pointFormat);

  // Alone among the formats punto reads, LAS states its extent up front.
  info.boundsKnown = true;
  info.minBound = Vec3{header.minBound[0], header.minBound[1], header.minBound[2]};
  info.maxBound = Vec3{header.maxBound[0], header.maxBound[1], header.maxBound[2]};

  // LAZ decompresses into a full-size record buffer that stays resident
  // until the whole cloud is built.
  if (header.compressed) info.memory.decodeBuffer = naturalStride * header.pointCount;
  if (header.compressed && !lazSupported())
    info.warnings.push_back("LAZ support is not compiled in; this file cannot be opened");
  return true;
}

}  // namespace punto
