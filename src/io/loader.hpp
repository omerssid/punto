// Format detection and the common loader entry point.
#pragma once

#include <stop_token>
#include <string>
#include <vector>

#include "io/point_cloud.hpp"

namespace punto {

enum class CloudFormat { Unknown, Pcd, Ply, Las, Laz, KittiBin };

const char* cloudFormatName(CloudFormat f);

// Identifies a file by content first, falling back to its extension. Content
// sniffing matters because .bin and .pcd both get used loosely in practice.
CloudFormat detectFormat(const std::string& path);

struct LoadOptions {
  // Reject points whose coordinates are NaN or infinite. These appear as
  // "no return" placeholders in organised depth and LiDAR clouds; keeping
  // them would corrupt the bounding box and render as garbage.
  bool dropInvalid = true;

  // Record each point's row in the source file (4 bytes/point).
  bool keepSourceIndex = true;

  // 0 selects defaultWorkerCount().
  unsigned workers = 0;

  // Keep at most this many points, taking every Nth record from the file.
  // 0 keeps all of them.
  //
  // Subsampling at decode time rather than afterwards is the point: a cloud
  // thinned once it is already in memory has still cost its full peak, which
  // is exactly the peak that could not be afforded.
  std::uint64_t maxPoints = 0;

  // Scalar fields to leave out entirely, by name. They are neither decoded
  // nor given a column. GPS time alone is 8 bytes a point -- 700 MB on an
  // 87M-point file -- and a viewer never reads it.
  std::vector<std::string> dropFields;

  // For KITTI-style raw .bin, which carries no header to describe itself.
  // Ignored by every self-describing format.
  int rawBinFieldCount = 4;  // x y z intensity
};

struct LoadResult {
  bool ok = false;
  std::string error;
  std::vector<std::string> warnings;
  bool cancelled = false;

  static LoadResult failure(std::string msg) {
    LoadResult r;
    r.error = std::move(msg);
    return r;
  }
};

// Loads `path` into `out`. On success `out` holds positions, columns, bounds
// and per-field statistics. `stop` is polled during the decode so a mistaken
// open of a huge file can be abandoned.
LoadResult loadPointCloud(const std::string& path, const LoadOptions& options, PointCloud& out,
                          std::stop_token stop = {});

// Per-format entry points, exposed for tests and for callers that already
// know what they have.
LoadResult loadPcd(const std::string& path, const LoadOptions& options, PointCloud& out,
                   std::stop_token stop);
LoadResult loadPly(const std::string& path, const LoadOptions& options, PointCloud& out,
                   std::stop_token stop);
LoadResult loadLas(const std::string& path, const LoadOptions& options, PointCloud& out,
                   std::stop_token stop);
LoadResult loadKittiBin(const std::string& path, const LoadOptions& options, PointCloud& out,
                        std::stop_token stop);

}  // namespace punto
