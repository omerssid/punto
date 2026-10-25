#include "io/cloud_info.hpp"

#include <filesystem>

#include "io/point_cloud.hpp"
#include "scene/chunk.hpp"

namespace punto {
namespace {

// What the GPU holds per point, mirroring RamChunkSource's upload.
// static_assert keeps these honest if the vertex format ever changes.
constexpr std::uint64_t kGpuPositionBytes = sizeof(QuantizedPoint);
static_assert(kGpuPositionBytes == 8, "GPU position estimate is out of date");
constexpr std::uint64_t kGpuColorBytes = 4;   // packed RGBA8
constexpr std::uint64_t kGpuScalarBytes = 2;  // the active field, quantized to uint16

}  // namespace

MemoryEstimate estimateMemory(const std::vector<SourceField>& fields, std::uint64_t pointCount,
                              std::uint64_t decodeBufferBytes, const LoadOptions& options) {
  MemoryEstimate est;
  est.pointCount = pointCount;
  est.decodeBuffer = decodeBufferBytes;

  // Plan against a throwaway cloud. buildDecodePlan fills in column metadata
  // without allocating per-point storage, so this costs nothing and -- more
  // to the point -- cannot disagree with what the loader will actually do.
  PointCloud scratch;
  std::vector<std::string> ignored;
  const DecodePlan plan = buildDecodePlan(fields, scratch, ignored);

  std::uint64_t perPoint = sizeof(Vec3f);
  if (plan.producesColor) perPoint += sizeof(std::uint32_t);
  if (options.keepSourceIndex) perPoint += sizeof(std::uint32_t);
  // Take column widths from the plan entries rather than from scratch.columns:
  // buildDecodePlan only records which columns exist, and a column's type is
  // not set until allocateCloud resets it. Reading them here from the same
  // place allocateCloud does is what keeps the estimate honest.
  for (const DecodePlan::Entry& e : plan.entries)
    if (e.dest == FieldDest::Column && e.column >= 0) perPoint += fieldTypeSize(e.type);

  est.bytesPerPoint = static_cast<std::uint32_t>(perPoint);
  est.pointCloud = perPoint * pointCount;

  // Chunk building, as buildChunks allocates it: Morton keys and the radix
  // sort's scratch (8 bytes each), the permutation (4), and then one gather
  // temporary at a time -- the largest being positions. `keys` is released
  // before the gather starts, so the two peaks are distinct.
  const std::uint64_t sortPeak = (8 + 8 + 4) * pointCount;
  const std::uint64_t gatherPeak = (8 + 4) * pointCount + sizeof(Vec3f) * pointCount;
  est.chunkScratch = std::max(sortPeak, gatherPeak);

  est.gpuVertex = (kGpuPositionBytes + (plan.producesColor ? kGpuColorBytes : 0) +
                   kGpuScalarBytes) * pointCount;
  return est;
}

CloudInfo probeCloud(const std::string& path, const LoadOptions& options) {
  CloudInfo info;

  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    info.error = "no such file: " + path;
    return info;
  }
  info.fileBytes = std::filesystem::file_size(path, ec);
  if (ec) info.fileBytes = 0;

  info.format = detectFormat(path);
  info.formatName = cloudFormatName(info.format);

  bool parsed = false;
  switch (info.format) {
    case CloudFormat::Pcd:      parsed = probePcdHeader(path, info); break;
    case CloudFormat::Ply:      parsed = probePlyHeader(path, info); break;
    case CloudFormat::Las:
    case CloudFormat::Laz:      parsed = probeLasHeader(path, info); break;
    case CloudFormat::KittiBin: parsed = probeKittiBinHeader(path, options, info); break;
    case CloudFormat::Unknown:
      info.error = "unrecognised point cloud format: " + path;
      return info;
  }
  if (!parsed) return info;

  // Only a format that materialises decompressed records sets this; the rest
  // decode straight out of the mapping and pay nothing extra.
  const std::uint64_t decodeBuffer = info.memory.decodeBuffer;
  info.memory = estimateMemory(info.fields, info.pointCount, decodeBuffer, options);
  for (const SourceField& f : info.fields)
    if (f.name == "red" || f.name == "rgb" || f.name == "rgba") info.hasColor = true;

  info.ok = true;
  return info;
}

}  // namespace punto
