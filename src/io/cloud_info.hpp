// Header-only inspection: what a file holds, and what opening it will cost.
//
// Every loader sizes the whole cloud before it reports anything, so the tool
// meant to warn that a file is too large was itself the thing that ran the
// machine out of memory. probeCloud reads only the header, which is what
// makes a pre-flight check possible at all.
#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "core/math.hpp"
#include "io/cloud_builder.hpp"
#include "io/loader.hpp"

namespace punto {

// Predicted resident bytes for each stage of opening a cloud.
//
// The per-stage split matters: a file can fit in RAM as a PointCloud and
// still fail to open, because the decompressed source records and the
// finished cloud are briefly alive together.
struct MemoryEstimate {
  std::uint64_t pointCount = 0;
  std::uint32_t bytesPerPoint = 0;  // the PointCloud's own per-point cost

  std::uint64_t decodeBuffer = 0;   // decompressed records held during load
  std::uint64_t pointCloud = 0;     // positions + colours + source index + columns
  std::uint64_t chunkScratch = 0;   // transient peak while building chunks
  std::uint64_t gpuVertex = 0;      // VRAM after upload

  // Loading holds the decode buffer and the finished cloud at once; chunking
  // holds the cloud and its scratch. The peak is whichever is worse.
  std::uint64_t peakHost() const {
    return std::max(decodeBuffer + pointCloud, pointCloud + chunkScratch);
  }

  // Allocator slack, resident pages of the source mapping, chunk metadata and
  // the LAZ decompressor's internal buffers are not modelled individually.
  // Against real files the stage figures come out a few percent light, so
  // anything deciding whether to proceed works from the padded number --
  // a guard that is optimistic is worse than no guard at all.
  static constexpr double kOverheadFactor = 1.15;
  std::uint64_t peakHostPadded() const {
    return static_cast<std::uint64_t>(static_cast<double>(peakHost()) * kOverheadFactor);
  }
};

// What a file's header declares, with no point data read.
struct CloudInfo {
  bool ok = false;
  std::string error;
  std::vector<std::string> warnings;

  CloudFormat format = CloudFormat::Unknown;
  std::string formatName;
  std::uint64_t fileBytes = 0;
  std::uint64_t pointCount = 0;
  std::size_t recordStride = 0;

  // Fields exactly as the header declares them. Nothing is inferred from
  // the data, so a field's range is unknown here -- that needs a full load.
  std::vector<SourceField> fields;
  bool hasColor = false;

  // LAS states its bounding box in the header; PCD, PLY and raw .bin do not,
  // so bounds stay unknown until the points are read.
  bool boundsKnown = false;
  Vec3 minBound{0.0, 0.0, 0.0};
  Vec3 maxBound{0.0, 0.0, 0.0};

  MemoryEstimate memory;
};

// Reads `path`'s header and predicts the cost of opening it. Touches a few
// kilobytes regardless of file size.
CloudInfo probeCloud(const std::string& path, const LoadOptions& options);

// Fills `memory` from a field list and point count. Exposed so a loader that
// has already parsed its header can estimate without re-reading the file.
MemoryEstimate estimateMemory(const std::vector<SourceField>& fields, std::uint64_t pointCount,
                              std::uint64_t decodeBufferBytes, const LoadOptions& options);

// Per-format header probes, each defined alongside its loader so that header
// parsing is never written twice. `info.format` and `info.fileBytes` are set
// by probeCloud before dispatch.
bool probePcdHeader(const std::string& path, CloudInfo& info);
bool probePlyHeader(const std::string& path, CloudInfo& info);
bool probeLasHeader(const std::string& path, CloudInfo& info);
bool probeKittiBinHeader(const std::string& path, const LoadOptions& options, CloudInfo& info);

}  // namespace punto
