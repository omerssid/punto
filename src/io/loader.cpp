#include "io/loader.hpp"

#include <algorithm>
#include <format>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <string_view>

#include "core/log.hpp"
#include "io/cloud_builder.hpp"
#include "io/cloud_info.hpp"
#include "io/mapped_file.hpp"

namespace punto {
namespace {

std::string lowerExtension(const std::string& path) {
  std::string ext = std::filesystem::path(path).extension().string();
  for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return ext;
}

// Peeks at the first bytes of a file without mapping the whole thing.
std::string readPrefix(const std::string& path, std::size_t n) {
  std::string buf;
  if (std::FILE* f = std::fopen(path.c_str(), "rb")) {
    buf.resize(n);
    const std::size_t got = std::fread(buf.data(), 1, n, f);
    buf.resize(got);
    std::fclose(f);
  }
  return buf;
}

bool startsWith(std::string_view s, std::string_view prefix) {
  return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

// Raw .bin is headerless: a flat array of float32 tuples. KITTI uses 4
// (x y z intensity), nuScenes 5 (adding ring), and some tools emit 3 or 6.
// Prefer the requested count, falling back to whatever divides the file size
// evenly. Returns 0 when nothing does.
int inferBinFieldCount(std::uint64_t fileBytes, int requested) {
  requested = std::clamp(requested, 3, 8);
  for (int candidate : {requested, 4, 3, 5, 6, 2})
    if (candidate >= 3 && fileBytes % (sizeof(float) * static_cast<std::size_t>(candidate)) == 0)
      return candidate;
  return 0;
}

// Names punto gives the columns of a headerless .bin.
const char* kBinFieldNames[] = {"x",      "y",      "z",      "intensity",
                                "field4", "field5", "field6", "field7"};

}  // namespace

const char* cloudFormatName(CloudFormat f) {
  switch (f) {
    case CloudFormat::Pcd:      return "PCD";
    case CloudFormat::Ply:      return "PLY";
    case CloudFormat::Las:      return "LAS";
    case CloudFormat::Laz:      return "LAZ";
    case CloudFormat::KittiBin: return "raw binary XYZI";
    case CloudFormat::Unknown:  return "unknown";
  }
  return "unknown";
}

CloudFormat detectFormat(const std::string& path) {
  // Content first: .bin and .pcd both get applied loosely, and a mislabelled
  // file is better identified by what it actually contains.
  const std::string prefix = readPrefix(path, 256);

  if (startsWith(prefix, "ply")) return CloudFormat::Ply;
  if (startsWith(prefix, "LASF")) return CloudFormat::Las;
  // LAZ is a LAS file whose point records are compressed; the magic matches,
  // so the distinction comes from the extension and from a VLR we read later.
  if (prefix.find("# .PCD") != std::string::npos || startsWith(prefix, "VERSION"))
    return CloudFormat::Pcd;

  const std::string ext = lowerExtension(path);
  if (ext == ".pcd") return CloudFormat::Pcd;
  if (ext == ".ply") return CloudFormat::Ply;
  if (ext == ".las") return CloudFormat::Las;
  if (ext == ".laz") return CloudFormat::Laz;
  if (ext == ".bin") return CloudFormat::KittiBin;
  return CloudFormat::Unknown;
}

LoadResult loadKittiBin(const std::string& path, const LoadOptions& options, PointCloud& out,
                        std::stop_token stop) {
  ScopedTimer totalTimer("loadKittiBin");
  LoadResult result;
  out.clear();

  Stopwatch mapTimer;
  MappedFile file;
  std::string error;
  if (!file.open(path, error)) return LoadResult::failure(std::move(error));
  out.timing.mapMs = mapTimer.elapsedMs();

  const int requested = std::clamp(options.rawBinFieldCount, 3, 8);
  const int fieldCount = inferBinFieldCount(file.size(), requested);
  if (fieldCount == 0)
    return LoadResult::failure("raw .bin: size " + std::to_string(file.size()) +
                               " is not a whole number of float32 tuples");
  if (fieldCount != requested)
    result.warnings.push_back("raw .bin: assuming " + std::to_string(fieldCount) +
                              " float32 fields per point based on file size");

  std::vector<SourceField> fields;
  for (int i = 0; i < fieldCount; ++i)
    fields.push_back(SourceField{kBinFieldNames[i], FieldType::F32, 1, 0});

  const std::size_t recordStride = layoutFields(fields);
  const std::size_t pointCount = file.size() / recordStride;

  DecodePlan plan = buildDecodePlan(fields, out, result.warnings);
  setInterleavedStride(plan, recordStride);

  out.sourcePath = path;
  out.sourceBytes = file.size();
  out.formatName = "raw binary XYZI";
  out.width = static_cast<std::uint32_t>(pointCount);
  out.height = 1;

  file.adviseSequential(0, file.size());
  const auto* base = reinterpret_cast<const std::byte*>(file.chars());

  Stopwatch decodeTimer;
  allocateCloud(out, plan, pointCount, options);
  out.origin = chooseOrigin(plan, base, pointCount);

  std::uint64_t invalid = 0;
  if (!decodeParallel(plan, base, pointCount, out.origin, options, std::move(stop), out, invalid)) {
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

// The path is unused: a headerless .bin says everything in its size alone.
bool probeKittiBinHeader(const std::string&, const LoadOptions& options, CloudInfo& info) {
  const int requested = std::clamp(options.rawBinFieldCount, 3, 8);
  const int fieldCount = inferBinFieldCount(info.fileBytes, requested);
  if (fieldCount == 0) {
    info.error = "raw .bin: size " + std::to_string(info.fileBytes) +
                 " is not a whole number of float32 tuples";
    return false;
  }
  if (fieldCount != requested)
    info.warnings.push_back("raw .bin: assuming " + std::to_string(fieldCount) +
                            " float32 fields per point based on file size");

  for (int i = 0; i < fieldCount; ++i)
    info.fields.push_back(SourceField{kBinFieldNames[i], FieldType::F32, 1, 0});
  info.recordStride = layoutFields(info.fields);
  info.pointCount = info.recordStride > 0 ? info.fileBytes / info.recordStride : 0;
  info.formatName = std::format("raw binary, {} float32 fields", fieldCount);
  // Decoded straight out of the mapping.
  return true;
}

LoadResult loadPointCloud(const std::string& path, const LoadOptions& options, PointCloud& out,
                          std::stop_token stop) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec))
    return LoadResult::failure("no such file: " + path);

  const CloudFormat format = detectFormat(path);
  logInfo("opening {} as {}", path, cloudFormatName(format));

  switch (format) {
    case CloudFormat::Pcd:      return loadPcd(path, options, out, std::move(stop));
    case CloudFormat::Ply:      return loadPly(path, options, out, std::move(stop));
    case CloudFormat::Las:
    case CloudFormat::Laz:      return loadLas(path, options, out, std::move(stop));
    case CloudFormat::KittiBin: return loadKittiBin(path, options, out, std::move(stop));
    case CloudFormat::Unknown:  break;
  }
  return LoadResult::failure("unrecognised point cloud format: " + path);
}

}  // namespace punto
