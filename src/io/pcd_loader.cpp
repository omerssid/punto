// PCD reader covering ascii, binary and binary_compressed payloads.
//
// Written against the format rather than against PCL: arbitrary FIELDS are
// carried through as typed columns instead of being forced into one of a
// handful of fixed point structs, and the decode runs across every core.
#include <algorithm>
#include <atomic>
#include <cctype>
#include <charconv>
#include <cstring>
#include <string_view>
#include <vector>

#include "core/log.hpp"
#include "core/parallel.hpp"
#include "io/cloud_builder.hpp"
#include "io/cloud_info.hpp"
#include "io/lzf.hpp"
#include "io/mapped_file.hpp"

namespace punto {
namespace {

enum class PcdData { Ascii, Binary, BinaryCompressed };

struct PcdHeader {
  std::vector<SourceField> fields;
  std::uint64_t width = 0;
  std::uint64_t height = 1;
  std::uint64_t points = 0;
  PcdData data = PcdData::Ascii;
  std::size_t payloadOffset = 0;
  std::string version;
};

std::vector<std::string_view> splitWhitespace(std::string_view line) {
  std::vector<std::string_view> out;
  std::size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
    const std::size_t start = i;
    while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))) ++i;
    if (i > start) out.push_back(line.substr(start, i - start));
  }
  return out;
}

std::string upperAscii(std::string_view s) {
  std::string o(s);
  for (char& c : o) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return o;
}

std::string lowerAscii(std::string_view s) {
  std::string o(s);
  for (char& c : o) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return o;
}

bool parseU64(std::string_view s, std::uint64_t& out) {
  return std::from_chars(s.data(), s.data() + s.size(), out).ec == std::errc{};
}

// Parses the text header. Returns false with `error` set on a malformed file.
bool parseHeader(std::string_view text, PcdHeader& header, std::string& error,
                 std::vector<std::string>& warnings) {
  std::vector<std::string_view> fieldNames;
  std::vector<std::uint64_t> sizes;
  std::vector<std::string_view> types;
  std::vector<std::uint64_t> counts;

  std::size_t pos = 0;
  while (pos < text.size()) {
    std::size_t eol = text.find('\n', pos);
    if (eol == std::string_view::npos) eol = text.size();
    std::string_view line = text.substr(pos, eol - pos);
    if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
    const std::size_t lineStart = pos;
    pos = eol + 1;

    if (line.empty() || line[0] == '#') continue;

    const auto tok = splitWhitespace(line);
    if (tok.empty()) continue;
    const std::string key = upperAscii(tok[0]);

    if (key == "VERSION") {
      if (tok.size() > 1) header.version = std::string(tok[1]);
    } else if (key == "FIELDS" || key == "COLUMNS") {
      fieldNames.assign(tok.begin() + 1, tok.end());
    } else if (key == "SIZE") {
      sizes.clear();
      for (std::size_t i = 1; i < tok.size(); ++i) {
        std::uint64_t v = 0;
        if (!parseU64(tok[i], v)) { error = "PCD: bad SIZE entry"; return false; }
        sizes.push_back(v);
      }
    } else if (key == "TYPE") {
      types.assign(tok.begin() + 1, tok.end());
    } else if (key == "COUNT") {
      counts.clear();
      for (std::size_t i = 1; i < tok.size(); ++i) {
        std::uint64_t v = 0;
        if (!parseU64(tok[i], v)) { error = "PCD: bad COUNT entry"; return false; }
        counts.push_back(v);
      }
    } else if (key == "WIDTH") {
      if (tok.size() < 2 || !parseU64(tok[1], header.width)) { error = "PCD: bad WIDTH"; return false; }
    } else if (key == "HEIGHT") {
      if (tok.size() < 2 || !parseU64(tok[1], header.height)) { error = "PCD: bad HEIGHT"; return false; }
    } else if (key == "POINTS") {
      if (tok.size() < 2 || !parseU64(tok[1], header.points)) { error = "PCD: bad POINTS"; return false; }
    } else if (key == "DATA") {
      if (tok.size() < 2) { error = "PCD: DATA has no value"; return false; }
      const std::string v = lowerAscii(tok[1]);
      if (v == "ascii") header.data = PcdData::Ascii;
      else if (v == "binary") header.data = PcdData::Binary;
      else if (v == "binary_compressed") header.data = PcdData::BinaryCompressed;
      else { error = "PCD: unsupported DATA '" + v + "'"; return false; }
      // The payload begins immediately after this line's newline.
      header.payloadOffset = pos;
      (void)lineStart;
      break;
    }
    // VIEWPOINT is parsed but unused; the viewer shows clouds in their own frame.
  }

  if (header.payloadOffset == 0) { error = "PCD: no DATA line found"; return false; }
  if (fieldNames.empty()) { error = "PCD: no FIELDS declared"; return false; }
  if (sizes.size() != fieldNames.size() || types.size() != fieldNames.size()) {
    error = "PCD: FIELDS/SIZE/TYPE lengths disagree";
    return false;
  }
  if (!counts.empty() && counts.size() != fieldNames.size()) {
    warnings.push_back("PCD: COUNT length disagrees with FIELDS; assuming 1");
    counts.clear();
  }

  if (header.points == 0) header.points = header.width * std::max<std::uint64_t>(1, header.height);

  for (std::size_t i = 0; i < fieldNames.size(); ++i) {
    SourceField f;
    f.name = std::string(fieldNames[i]);
    f.count = counts.empty() ? 1 : std::max<std::uint64_t>(1, counts[i]);
    if (types[i].empty() || !fieldTypeFromPcd(types[i][0], static_cast<std::size_t>(sizes[i]), f.type)) {
      error = "PCD: field '" + f.name + "' has unsupported TYPE/SIZE " +
              std::string(types[i]) + "/" + std::to_string(sizes[i]);
      return false;
    }
    // PCL pads unused fields with the name "_"; they carry no data worth showing.
    if (f.name == "_") continue;
    header.fields.push_back(std::move(f));
  }
  if (header.fields.empty()) { error = "PCD: all fields are padding"; return false; }
  return true;
}

// Parses one whitespace-separated token into the destination type.
bool parseTokenInto(std::string_view tok, FieldType type, std::byte* dst) {
  // NaN and Inf appear routinely in ASCII clouds from organised sensors.
  auto writeDouble = [&](double v) {
    switch (type) {
      case FieldType::I8:  { auto t = static_cast<std::int8_t>(v);  std::memcpy(dst, &t, 1); break; }
      case FieldType::I16: { auto t = static_cast<std::int16_t>(v); std::memcpy(dst, &t, 2); break; }
      case FieldType::I32: { auto t = static_cast<std::int32_t>(v); std::memcpy(dst, &t, 4); break; }
      case FieldType::I64: { auto t = static_cast<std::int64_t>(v); std::memcpy(dst, &t, 8); break; }
      case FieldType::U8:  { auto t = static_cast<std::uint8_t>(v);  std::memcpy(dst, &t, 1); break; }
      case FieldType::U16: { auto t = static_cast<std::uint16_t>(v); std::memcpy(dst, &t, 2); break; }
      case FieldType::U32: { auto t = static_cast<std::uint32_t>(v); std::memcpy(dst, &t, 4); break; }
      case FieldType::U64: { auto t = static_cast<std::uint64_t>(v); std::memcpy(dst, &t, 8); break; }
      case FieldType::F32: { auto t = static_cast<float>(v);  std::memcpy(dst, &t, 4); break; }
      case FieldType::F64: { std::memcpy(dst, &v, 8); break; }
    }
  };

  double value = 0.0;
  const auto res = std::from_chars(tok.data(), tok.data() + tok.size(), value);
  if (res.ec != std::errc{}) {
    // from_chars handles "nan"/"inf" in libstdc++, but be explicit for the
    // signed and capitalised spellings that show up in the wild.
    const std::string low = lowerAscii(tok);
    if (low.find("nan") != std::string::npos) {
      value = std::numeric_limits<double>::quiet_NaN();
    } else if (low.find("inf") != std::string::npos) {
      value = low[0] == '-' ? -std::numeric_limits<double>::infinity()
                            : std::numeric_limits<double>::infinity();
    } else {
      return false;
    }
  }
  writeDouble(value);
  return true;
}

// ASCII payloads are decoded into a scratch interleaved record buffer and
// then run through the same plan as a binary file. This keeps one decode
// path for every encoding at the cost of one extra pass over the points.
bool decodeAscii(std::string_view payload, const PcdHeader& header, std::size_t recordStride,
                 std::size_t pointCount, const LoadOptions& options, std::stop_token stop,
                 std::vector<std::byte>& records, std::uint64_t& parsedCount,
                 std::vector<std::string>& warnings) {
  records.assign(recordStride * pointCount, std::byte{0});

  // Split the text into one byte range per worker, each starting at a line
  // boundary. Because a worker cannot know how many lines precede it, each
  // records its points locally and the blocks are concatenated afterwards.
  const unsigned workers = options.workers != 0 ? options.workers : defaultWorkerCount();
  std::vector<std::vector<std::byte>> local(workers);
  std::vector<std::uint64_t> localCount(workers, 0);
  std::atomic<bool> cancelled{false};
  std::atomic<std::uint64_t> badLines{0};

  parallelRangesCancellable(
      payload.size(), workers, std::move(stop),
      [&](std::size_t begin, std::size_t end, unsigned w, const std::stop_token& token) {
        std::size_t cursor = begin;
        // Every worker but the first resumes after the next newline, since
        // the line straddling its start belongs to the previous worker.
        if (w != 0 && cursor < payload.size()) {
          const std::size_t nl = payload.find('\n', cursor);
          cursor = (nl == std::string_view::npos) ? payload.size() : nl + 1;
        }

        std::vector<std::byte> buf;
        std::byte* record = nullptr;
        std::vector<std::byte> scratch(recordStride);
        std::uint64_t count = 0;
        std::uint64_t bad = 0;
        std::size_t sinceCheck = 0;

        while (cursor < end && cursor < payload.size()) {
          if (++sinceCheck >= 4096) {
            sinceCheck = 0;
            if (token.stop_requested()) { cancelled.store(true, std::memory_order_relaxed); return; }
          }
          std::size_t eol = payload.find('\n', cursor);
          if (eol == std::string_view::npos) eol = payload.size();
          std::string_view line = payload.substr(cursor, eol - cursor);
          cursor = eol + 1;
          if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
          if (line.empty() || line[0] == '#') continue;

          const auto tok = splitWhitespace(line);
          std::size_t expected = 0;
          for (const SourceField& f : header.fields) expected += std::max<std::size_t>(1, f.count);
          if (tok.size() < expected) { ++bad; continue; }

          std::fill(scratch.begin(), scratch.end(), std::byte{0});
          std::size_t t = 0;
          bool ok = true;
          for (const SourceField& f : header.fields) {
            const std::size_t count_ = std::max<std::size_t>(1, f.count);
            const std::size_t esz = fieldTypeSize(f.type);
            for (std::size_t c = 0; c < count_; ++c, ++t) {
              if (!parseTokenInto(tok[t], f.type, scratch.data() + f.byteOffset + c * esz)) {
                ok = false;
                break;
              }
            }
            if (!ok) break;
          }
          if (!ok) { ++bad; continue; }

          buf.insert(buf.end(), scratch.begin(), scratch.end());
          ++count;
        }

        local[w] = std::move(buf);
        localCount[w] = count;
        badLines.fetch_add(bad, std::memory_order_relaxed);
        (void)record;
      });

  if (cancelled.load(std::memory_order_relaxed)) return false;

  std::uint64_t total = 0;
  for (std::uint64_t c : localCount) total += c;
  if (total > pointCount) {
    warnings.push_back("PCD: file holds more points than POINTS declared; extra rows ignored");
    total = pointCount;
  } else if (total < pointCount) {
    warnings.push_back("PCD: file holds " + std::to_string(total) + " points but POINTS declared " +
                       std::to_string(pointCount));
  }

  records.assign(recordStride * total, std::byte{0});
  std::size_t out = 0;
  for (unsigned w = 0; w < workers && out < total; ++w) {
    const std::size_t avail = std::min<std::size_t>(localCount[w], total - out);
    if (avail > 0) std::memcpy(records.data() + out * recordStride, local[w].data(), avail * recordStride);
    out += avail;
  }

  const std::uint64_t bad = badLines.load(std::memory_order_relaxed);
  if (bad > 0) warnings.push_back("PCD: skipped " + std::to_string(bad) + " unparseable rows");

  parsedCount = total;
  return true;
}

}  // namespace

LoadResult loadPcd(const std::string& path, const LoadOptions& options, PointCloud& out,
                   std::stop_token stop) {
  ScopedTimer totalTimer("loadPcd");
  LoadResult result;
  out.clear();

  Stopwatch mapTimer;
  MappedFile file;
  std::string error;
  if (!file.open(path, error)) return LoadResult::failure(std::move(error));
  out.timing.mapMs = mapTimer.elapsedMs();

  Stopwatch headerTimer;
  PcdHeader header;
  // The header is text and short; cap the search so a binary blob
  // masquerading as a PCD does not get scanned end to end.
  const std::size_t headerSearch = std::min<std::size_t>(file.size(), 64u * 1024u);
  if (!parseHeader(std::string_view(file.chars(), headerSearch), header, error, result.warnings))
    return LoadResult::failure(std::move(error));
  out.timing.headerMs = headerTimer.elapsedMs();

  const std::size_t recordStride = layoutFields(header.fields);
  if (recordStride == 0) return LoadResult::failure("PCD: computed record stride is zero");

  DecodePlan plan = buildDecodePlan(header.fields, out, result.warnings);
  if (!plan.hasAllPositions())
    return LoadResult::failure("PCD: file declares no x/y/z fields");

  const std::size_t pointCount = static_cast<std::size_t>(header.points);
  if (pointCount == 0) return LoadResult::failure("PCD: declares zero points");

  out.sourcePath = path;
  out.sourceBytes = file.size();
  out.width = static_cast<std::uint32_t>(header.width);
  out.height = static_cast<std::uint32_t>(header.height);
  out.organized = header.height > 1;

  // Storage that must outlive the decode for the non-interleaved encodings.
  std::vector<std::byte> scratch;
  const std::byte* base = nullptr;
  std::size_t actualPoints = pointCount;

  Stopwatch decodeTimer;
  switch (header.data) {
    case PcdData::Binary: {
      out.formatName = "PCD (binary)";
      const std::size_t need = recordStride * pointCount;
      const std::size_t have = file.size() - header.payloadOffset;
      if (have < need) {
        result.warnings.push_back("PCD: payload is " + std::to_string(have) + " bytes but " +
                                  std::to_string(need) + " expected; truncating");
        actualPoints = have / recordStride;
      }
      file.adviseSequential(header.payloadOffset, actualPoints * recordStride);
      base = reinterpret_cast<const std::byte*>(file.chars() + header.payloadOffset);
      setInterleavedStride(plan, recordStride);
      break;
    }

    case PcdData::BinaryCompressed: {
      out.formatName = "PCD (binary_compressed)";
      // Payload: uint32 compressed size, uint32 uncompressed size, then the
      // LZF stream. The decompressed bytes are laid out field block by field
      // block rather than interleaved.
      if (file.size() < header.payloadOffset + 8)
        return LoadResult::failure("PCD: compressed payload truncated");
      std::uint32_t compressedSize = 0, uncompressedSize = 0;
      std::memcpy(&compressedSize, file.chars() + header.payloadOffset, 4);
      std::memcpy(&uncompressedSize, file.chars() + header.payloadOffset + 4, 4);

      const std::size_t available = file.size() - header.payloadOffset - 8;
      if (compressedSize > available)
        return LoadResult::failure("PCD: compressed size exceeds file");

      scratch.resize(uncompressedSize);
      const auto written = lzfDecompress(
          reinterpret_cast<const std::uint8_t*>(file.chars() + header.payloadOffset + 8),
          compressedSize, reinterpret_cast<std::uint8_t*>(scratch.data()), uncompressedSize);
      if (written != uncompressedSize)
        return LoadResult::failure("PCD: LZF decompression failed");

      const std::size_t need = recordStride * pointCount;
      if (uncompressedSize < need) {
        result.warnings.push_back("PCD: decompressed payload shorter than declared POINTS");
        actualPoints = uncompressedSize / recordStride;
      }
      base = scratch.data();
      setFieldBlockStrides(plan, header.fields, actualPoints);
      break;
    }

    case PcdData::Ascii: {
      out.formatName = "PCD (ascii)";
      const std::string_view payload(file.chars() + header.payloadOffset,
                                     file.size() - header.payloadOffset);
      std::uint64_t parsed = 0;
      if (!decodeAscii(payload, header, recordStride, pointCount, options, stop, scratch, parsed,
                       result.warnings)) {
        result.cancelled = true;
        result.error = "load cancelled";
        return result;
      }
      actualPoints = static_cast<std::size_t>(parsed);
      base = scratch.data();
      setInterleavedStride(plan, recordStride);
      break;
    }
  }

  if (actualPoints == 0) return LoadResult::failure("PCD: no points decoded");

  allocateCloud(out, plan, actualPoints, options);
  out.origin = chooseOrigin(plan, base, actualPoints);

  std::uint64_t invalid = 0;
  if (!decodeParallel(plan, base, actualPoints, out.origin, options, stop, out, invalid)) {
    out.clear();
    result.cancelled = true;
    result.error = "load cancelled";
    return result;
  }
  out.timing.decodeMs = decodeTimer.elapsedMs();

  if (options.dropInvalid) {
    compactInvalidPoints(out, invalid);
  } else if (invalid > 0) {
    out.invalidPointsDropped = 0;
    result.warnings.push_back(std::to_string(invalid) + " points have non-finite coordinates");
  }

  out.recomputeBounds();

  Stopwatch statsTimer;
  out.computeAllStats();
  out.timing.statsMs = statsTimer.elapsedMs();

  out.timing.totalMs = totalTimer.elapsedMs();
  result.ok = true;
  return result;
}


bool probePcdHeader(const std::string& path, CloudInfo& info) {
  MappedFile file;
  std::string error;
  if (!file.open(path, error)) {
    info.error = std::move(error);
    return false;
  }

  PcdHeader header;
  const std::size_t headerSearch = std::min<std::size_t>(file.size(), 64u * 1024u);
  if (!parseHeader(std::string_view(file.chars(), headerSearch), header, info.error,
                   info.warnings))
    return false;

  info.recordStride = layoutFields(header.fields);
  info.pointCount = header.points;
  info.fields = std::move(header.fields);
  static const char* kEncoding[] = {"ascii", "binary", "binary_compressed"};
  info.formatName = std::string("PCD ") + kEncoding[static_cast<int>(header.data)];

  // Binary PCD decodes straight out of the mapping; the other two encodings
  // are first materialised as interleaved records.
  if (header.data != PcdData::Binary)
    info.memory.decodeBuffer = info.recordStride * info.pointCount;
  return true;
}

}  // namespace punto
