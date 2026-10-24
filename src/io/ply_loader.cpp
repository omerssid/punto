// PLY reader: ascii, binary_little_endian and binary_big_endian.
//
// Only the `vertex` element is loaded. Faces and other elements are parsed
// far enough to be skipped, because this is a point cloud viewer and a PLY's
// connectivity is not something it can show.
#include <algorithm>
#include <atomic>
#include <cctype>
#include <charconv>
#include <cstring>
#include <limits>
#include <string_view>
#include <vector>

#include "core/log.hpp"
#include "core/parallel.hpp"
#include "io/cloud_builder.hpp"
#include "io/mapped_file.hpp"

namespace punto {
namespace {

enum class PlyFormat { Ascii, BinaryLittleEndian, BinaryBigEndian };

struct PlyProperty {
  std::string name;
  FieldType type = FieldType::F32;
  // A list property stores a count followed by that many values. Only faces
  // normally use them, but a vertex element is allowed to.
  bool isList = false;
  FieldType countType = FieldType::U8;
};

struct PlyElement {
  std::string name;
  std::uint64_t count = 0;
  std::vector<PlyProperty> properties;

  bool hasLists() const {
    return std::any_of(properties.begin(), properties.end(),
                       [](const PlyProperty& p) { return p.isList; });
  }
  // Bytes per record, valid only when the element has no list properties.
  std::size_t fixedSize() const {
    std::size_t total = 0;
    for (const PlyProperty& p : properties) total += fieldTypeSize(p.type);
    return total;
  }
};

struct PlyHeader {
  PlyFormat format = PlyFormat::Ascii;
  std::vector<PlyElement> elements;
  std::size_t payloadOffset = 0;
};

std::string lower(std::string_view s) {
  std::string o(s);
  for (char& c : o) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return o;
}

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

bool plyTypeFromName(std::string_view name, FieldType& out) {
  const std::string n = lower(name);
  if (n == "char" || n == "int8") { out = FieldType::I8; return true; }
  if (n == "uchar" || n == "uint8") { out = FieldType::U8; return true; }
  if (n == "short" || n == "int16") { out = FieldType::I16; return true; }
  if (n == "ushort" || n == "uint16") { out = FieldType::U16; return true; }
  if (n == "int" || n == "int32") { out = FieldType::I32; return true; }
  if (n == "uint" || n == "uint32") { out = FieldType::U32; return true; }
  if (n == "float" || n == "float32") { out = FieldType::F32; return true; }
  if (n == "double" || n == "float64") { out = FieldType::F64; return true; }
  return false;
}

bool parseHeader(std::string_view text, PlyHeader& header, std::string& error) {
  std::size_t pos = 0;
  bool sawMagic = false;
  bool sawFormat = false;

  while (pos < text.size()) {
    std::size_t eol = text.find('\n', pos);
    if (eol == std::string_view::npos) eol = text.size();
    std::string_view line = text.substr(pos, eol - pos);
    if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
    pos = eol + 1;

    const auto tok = splitWhitespace(line);
    if (tok.empty()) continue;
    const std::string key = lower(tok[0]);

    if (!sawMagic) {
      if (key != "ply") { error = "PLY: missing magic"; return false; }
      sawMagic = true;
      continue;
    }

    if (key == "comment" || key == "obj_info") continue;

    if (key == "format") {
      if (tok.size() < 2) { error = "PLY: malformed format line"; return false; }
      const std::string f = lower(tok[1]);
      if (f == "ascii") header.format = PlyFormat::Ascii;
      else if (f == "binary_little_endian") header.format = PlyFormat::BinaryLittleEndian;
      else if (f == "binary_big_endian") header.format = PlyFormat::BinaryBigEndian;
      else { error = "PLY: unsupported format '" + f + "'"; return false; }
      sawFormat = true;
      continue;
    }

    if (key == "element") {
      if (tok.size() < 3) { error = "PLY: malformed element line"; return false; }
      PlyElement element;
      element.name = lower(tok[1]);
      if (std::from_chars(tok[2].data(), tok[2].data() + tok[2].size(), element.count).ec !=
          std::errc{}) {
        error = "PLY: bad element count";
        return false;
      }
      header.elements.push_back(std::move(element));
      continue;
    }

    if (key == "property") {
      if (header.elements.empty()) { error = "PLY: property before any element"; return false; }
      PlyProperty property;
      if (tok.size() >= 2 && lower(tok[1]) == "list") {
        if (tok.size() < 5) { error = "PLY: malformed list property"; return false; }
        property.isList = true;
        if (!plyTypeFromName(tok[2], property.countType) ||
            !plyTypeFromName(tok[3], property.type)) {
          error = "PLY: unsupported list property types";
          return false;
        }
        property.name = std::string(tok[4]);
      } else {
        if (tok.size() < 3) { error = "PLY: malformed property"; return false; }
        if (!plyTypeFromName(tok[1], property.type)) {
          error = "PLY: unsupported property type '" + std::string(tok[1]) + "'";
          return false;
        }
        property.name = std::string(tok[2]);
      }
      header.elements.back().properties.push_back(std::move(property));
      continue;
    }

    if (key == "end_header") {
      header.payloadOffset = pos;
      if (!sawFormat) { error = "PLY: no format line"; return false; }
      return true;
    }
  }
  error = "PLY: no end_header";
  return false;
}

// Skips elements declared before `vertex`, returning the byte offset of the
// vertex block. Fails on a preceding element with list properties, whose
// size cannot be computed without walking every record.
bool vertexBlockOffset(const PlyHeader& header, std::size_t vertexIndex, std::size_t& offset,
                       std::string& error) {
  offset = header.payloadOffset;
  for (std::size_t i = 0; i < vertexIndex; ++i) {
    const PlyElement& element = header.elements[i];
    if (element.hasLists()) {
      error = "PLY: element '" + element.name +
              "' precedes the vertices and has list properties, so the vertex block "
              "cannot be located without decoding it";
      return false;
    }
    offset += element.fixedSize() * static_cast<std::size_t>(element.count);
  }
  return true;
}

// ASCII payloads are parsed into an interleaved scratch buffer and then run
// through the same plan as a binary file, so there is one decode path.
bool decodeAscii(std::string_view payload, const PlyElement& vertex, std::size_t recordStride,
                 std::size_t pointCount, std::stop_token stop,
                 std::vector<std::byte>& records, std::uint64_t& parsed,
                 std::vector<std::string>& warnings) {
  records.assign(recordStride * pointCount, std::byte{0});

  auto writeValue = [](std::string_view token, FieldType type, std::byte* dst) {
    double value = 0.0;
    if (std::from_chars(token.data(), token.data() + token.size(), value).ec != std::errc{}) {
      const std::string low = lower(token);
      if (low.find("nan") != std::string::npos)
        value = std::numeric_limits<double>::quiet_NaN();
      else if (low.find("inf") != std::string::npos)
        value = low[0] == '-' ? -std::numeric_limits<double>::infinity()
                              : std::numeric_limits<double>::infinity();
      else
        return false;
    }
    switch (type) {
      case FieldType::I8:  { auto t = static_cast<std::int8_t>(value);  std::memcpy(dst, &t, 1); break; }
      case FieldType::I16: { auto t = static_cast<std::int16_t>(value); std::memcpy(dst, &t, 2); break; }
      case FieldType::I32: { auto t = static_cast<std::int32_t>(value); std::memcpy(dst, &t, 4); break; }
      case FieldType::I64: { auto t = static_cast<std::int64_t>(value); std::memcpy(dst, &t, 8); break; }
      case FieldType::U8:  { auto t = static_cast<std::uint8_t>(value);  std::memcpy(dst, &t, 1); break; }
      case FieldType::U16: { auto t = static_cast<std::uint16_t>(value); std::memcpy(dst, &t, 2); break; }
      case FieldType::U32: { auto t = static_cast<std::uint32_t>(value); std::memcpy(dst, &t, 4); break; }
      case FieldType::U64: { auto t = static_cast<std::uint64_t>(value); std::memcpy(dst, &t, 8); break; }
      case FieldType::F32: { auto t = static_cast<float>(value);  std::memcpy(dst, &t, 4); break; }
      case FieldType::F64: { std::memcpy(dst, &value, 8); break; }
    }
    return true;
  };

  // Vertex records are parsed sequentially. Unlike PCD, a PLY's vertex block
  // is followed by other elements, so a parallel split would need to know
  // where the block ends -- which is the thing being computed.
  std::size_t cursor = 0;
  std::uint64_t written = 0;
  std::uint64_t bad = 0;
  std::size_t sinceCheck = 0;

  while (written < pointCount && cursor < payload.size()) {
    if (++sinceCheck >= 4096) {
      sinceCheck = 0;
      if (stop.stop_requested()) return false;
    }
    std::size_t eol = payload.find('\n', cursor);
    if (eol == std::string_view::npos) eol = payload.size();
    std::string_view line = payload.substr(cursor, eol - cursor);
    cursor = eol + 1;
    if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
    if (line.empty()) continue;

    const auto tok = splitWhitespace(line);
    std::byte* record = records.data() + written * recordStride;
    std::size_t t = 0;
    std::size_t byteOffset = 0;
    bool ok = true;
    for (const PlyProperty& property : vertex.properties) {
      if (property.isList) {
        // Read the count, then skip that many values.
        if (t >= tok.size()) { ok = false; break; }
        std::uint64_t n = 0;
        std::from_chars(tok[t].data(), tok[t].data() + tok[t].size(), n);
        t += 1 + static_cast<std::size_t>(n);
        continue;
      }
      if (t >= tok.size()) { ok = false; break; }
      if (!writeValue(tok[t], property.type, record + byteOffset)) { ok = false; break; }
      byteOffset += fieldTypeSize(property.type);
      ++t;
    }
    if (!ok) { ++bad; continue; }
    ++written;
  }

  if (bad > 0) warnings.push_back("PLY: skipped " + std::to_string(bad) + " unparseable rows");
  parsed = written;
  return true;
}

}  // namespace

LoadResult loadPly(const std::string& path, const LoadOptions& options, PointCloud& out,
                   std::stop_token stop) {
  ScopedTimer totalTimer("loadPly");
  LoadResult result;
  out.clear();

  Stopwatch mapTimer;
  MappedFile file;
  std::string error;
  if (!file.open(path, error)) return LoadResult::failure(std::move(error));
  out.timing.mapMs = mapTimer.elapsedMs();

  Stopwatch headerTimer;
  PlyHeader header;
  const std::size_t headerSearch = std::min<std::size_t>(file.size(), 256u * 1024u);
  if (!parseHeader(std::string_view(file.chars(), headerSearch), header, error))
    return LoadResult::failure(std::move(error));
  out.timing.headerMs = headerTimer.elapsedMs();

  std::size_t vertexIndex = header.elements.size();
  for (std::size_t i = 0; i < header.elements.size(); ++i)
    if (header.elements[i].name == "vertex") vertexIndex = i;
  if (vertexIndex == header.elements.size())
    return LoadResult::failure("PLY: no 'vertex' element");

  const PlyElement& vertex = header.elements[vertexIndex];
  if (vertex.count == 0) return LoadResult::failure("PLY: vertex element is empty");

  // List properties on the vertex element are legal but vanishingly rare,
  // and would make every record a different size.
  if (header.format != PlyFormat::Ascii && vertex.hasLists())
    return LoadResult::failure("PLY: binary vertex elements with list properties "
                               "are not supported");

  std::vector<SourceField> fields;
  for (const PlyProperty& property : vertex.properties) {
    if (property.isList) continue;  // handled by skipping in the ASCII path
    fields.push_back(SourceField{property.name, property.type, 1, 0});
  }
  if (fields.empty()) return LoadResult::failure("PLY: vertex element has no scalar properties");

  const std::size_t recordStride = layoutFields(fields);
  DecodePlan plan = buildDecodePlan(fields, out, result.warnings);
  if (!plan.hasAllPositions())
    return LoadResult::failure("PLY: vertex element declares no x/y/z properties");

  setInterleavedStride(plan, recordStride);
  if (header.format == PlyFormat::BinaryBigEndian)
    for (DecodePlan::Entry& e : plan.entries) e.byteSwap = true;

  out.sourcePath = path;
  out.sourceBytes = file.size();
  out.width = static_cast<std::uint32_t>(vertex.count);
  out.height = 1;

  std::vector<std::byte> scratch;
  const std::byte* base = nullptr;
  std::size_t pointCount = static_cast<std::size_t>(vertex.count);

  Stopwatch decodeTimer;
  if (header.format == PlyFormat::Ascii) {
    out.formatName = "PLY (ascii)";
    const std::string_view payload(file.chars() + header.payloadOffset,
                                   file.size() - header.payloadOffset);
    std::uint64_t parsed = 0;
    if (!decodeAscii(payload, vertex, recordStride, pointCount, stop, scratch, parsed,
                     result.warnings)) {
      result.cancelled = true;
      result.error = "load cancelled";
      return result;
    }
    pointCount = static_cast<std::size_t>(parsed);
    base = scratch.data();
  } else {
    out.formatName = header.format == PlyFormat::BinaryBigEndian ? "PLY (binary big-endian)"
                                                                 : "PLY (binary little-endian)";
    std::size_t offset = 0;
    if (!vertexBlockOffset(header, vertexIndex, offset, error))
      return LoadResult::failure(std::move(error));

    const std::size_t need = recordStride * pointCount;
    if (offset + need > file.size()) {
      const std::size_t available = file.size() > offset ? file.size() - offset : 0;
      result.warnings.push_back("PLY: vertex block is truncated; reading " +
                                std::to_string(available / recordStride) + " of " +
                                std::to_string(pointCount) + " points");
      pointCount = available / recordStride;
    }
    if (pointCount == 0) return LoadResult::failure("PLY: no complete vertex records");
    file.adviseSequential(offset, pointCount * recordStride);
    base = reinterpret_cast<const std::byte*>(file.chars() + offset);
  }

  allocateCloud(out, plan, pointCount, options);
  out.origin = chooseOrigin(plan, base, pointCount);

  std::uint64_t invalid = 0;
  if (!decodeParallel(plan, base, pointCount, out.origin, options, stop, out, invalid)) {
    out.clear();
    result.cancelled = true;
    result.error = "load cancelled";
    return result;
  }
  out.timing.decodeMs = decodeTimer.elapsedMs();

  if (options.dropInvalid) compactInvalidPoints(out, invalid);
  out.recomputeBounds();

  Stopwatch statsTimer;
  out.computeAllStats();
  out.timing.statsMs = statsTimer.elapsedMs();

  out.timing.totalMs = totalTimer.elapsedMs();
  result.ok = true;
  return result;
}

}  // namespace punto
