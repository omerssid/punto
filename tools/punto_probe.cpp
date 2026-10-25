// punto-probe -- inspect a point cloud from the terminal.
//
// Prints what pcl_viewer never tells you: every declared field with its type,
// range, mean and distribution; the exact bounding box; invalid point counts;
// and where the load time actually went.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <format>
#include <string>
#include <vector>

#include "core/log.hpp"
#include "core/parallel.hpp"
#include "core/memory.hpp"
#include "io/cloud_info.hpp"
#include "io/loader.hpp"
#include "scene/chunk.hpp"
#include "scene/ram_chunk_source.hpp"

namespace {

using namespace punto;

// Renders a histogram as one line of block characters.
std::string sparkline(const std::vector<std::uint32_t>& bins, std::size_t width) {
  static const char* kBlocks[] = {" ", "▁", "▂", "▃",
                                  "▄", "▅", "▆", "▇"};
  if (bins.empty()) return {};

  // Resample the 256 bins down to `width` columns.
  std::vector<std::uint64_t> cols(width, 0);
  for (std::size_t i = 0; i < bins.size(); ++i)
    cols[i * width / bins.size()] += bins[i];

  std::uint64_t peak = 0;
  for (std::uint64_t v : cols) peak = std::max(peak, v);
  if (peak == 0) return std::string(width, ' ');

  std::string out;
  for (std::uint64_t v : cols) {
    // Log scale, so a long tail stays visible next to a dominant mode.
    const double norm = v == 0 ? 0.0 : std::log1p(static_cast<double>(v)) / std::log1p(static_cast<double>(peak));
    const int level = std::min(7, static_cast<int>(norm * 7.999));
    out += kBlocks[level];
  }
  return out;
}

std::string humanBytes(std::uint64_t bytes) {
  const char* units[] = {"B", "KB", "MB", "GB", "TB"};
  double v = static_cast<double>(bytes);
  int u = 0;
  while (v >= 1024.0 && u < 4) { v /= 1024.0; ++u; }
  return std::format("{:.1f} {}", v, units[u]);
}

std::string withThousands(std::uint64_t v) {
  std::string s = std::to_string(v);
  for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3) s.insert(static_cast<std::size_t>(i), ",");
  return s;
}

void printReport(const PointCloud& cloud, const LoadResult& result) {
  std::printf("\n\x1b[1m%s\x1b[0m\n", cloud.sourcePath.c_str());
  std::printf("  format        %s\n", cloud.formatName.c_str());
  std::printf("  file size     %s\n", humanBytes(cloud.sourceBytes).c_str());
  std::printf("  points        %s\n", withThousands(cloud.size()).c_str());
  if (cloud.invalidPointsDropped > 0) {
    const double pct = 100.0 * static_cast<double>(cloud.invalidPointsDropped) /
                       static_cast<double>(cloud.invalidPointsDropped + cloud.size());
    std::printf("  invalid       %s dropped (%.2f%%)\n",
                withThousands(cloud.invalidPointsDropped).c_str(), pct);
  }
  if (cloud.organized) std::printf("  organized     %u x %u\n", cloud.width, cloud.height);
  std::printf("  resident      %s\n", humanBytes(cloud.memoryBytes()).c_str());
  std::printf("  colors        %s\n", cloud.hasColors() ? "yes" : "no");
  std::printf("  source rows   %s\n", cloud.hasSourceIndex() ? "tracked" : "not tracked");

  const Aabb& b = cloud.boundsWorld;
  if (b.valid()) {
    const Vec3 size = b.size();
    const Vec3 c = b.center();
    std::printf("\n  \x1b[1mbounds (world)\x1b[0m\n");
    std::printf("    min   %16.4f %16.4f %16.4f\n", b.lo.x, b.lo.y, b.lo.z);
    std::printf("    max   %16.4f %16.4f %16.4f\n", b.hi.x, b.hi.y, b.hi.z);
    std::printf("    size  %16.4f %16.4f %16.4f\n", size.x, size.y, size.z);
    std::printf("    mid   %16.4f %16.4f %16.4f\n", c.x, c.y, c.z);
    std::printf("    diag  %.4f m\n", b.diagonal());
  }
  std::printf("  origin  %16.4f %16.4f %16.4f   (positions stored relative to this)\n",
              cloud.origin.x, cloud.origin.y, cloud.origin.z);

  std::printf("\n  \x1b[1mfields\x1b[0m\n");
  std::printf("    %-14s %-9s %-10s %14s %14s %14s %12s  %s\n", "name", "type", "role", "min",
              "max", "mean", "stddev", "distribution");
  std::printf("    %s\n", std::string(14 + 9 + 10 + 14 * 3 + 12 + 20, '-').c_str());

  for (const FieldInfo& f : cloud.fields) {
    const FieldStats& s = f.stats;
    if (!s.valid()) {
      // Packed rgb/rgba folds into the colour array, so it has no scalar
      // column; its distribution is reported per channel just below.
      const char* note = (f.role == FieldRole::Color && cloud.colorStatsValid)
                             ? "packed -> colour channels below"
                             : "(no data)";
      std::printf("    %-14s %-9s %-10s %s\n", f.name.c_str(), fieldTypeName(f.type),
                  fieldRoleName(f.role), note);
      continue;
    }
    std::printf("    %-14s %-9s %-10s %14.4f %14.4f %14.4f %12.4f  %s\n", f.name.c_str(),
                fieldTypeName(f.type), fieldRoleName(f.role), s.min, s.max, s.mean, s.stddev,
                sparkline(s.histogram, 24).c_str());
    if (s.nonFiniteCount > 0)
      std::printf("    %-14s   \x1b[33m%s non-finite values\x1b[0m\n", "",
                  withThousands(s.nonFiniteCount).c_str());
    if (s.discrete && !s.distinctValues.empty()) {
      std::string list;
      for (std::size_t i = 0; i < s.distinctValues.size() && i < 16; ++i) {
        if (i) list += " ";
        list += std::format("{:g}", s.distinctValues[i]);
      }
      if (s.distinctValues.size() > 16) list += " ...";
      std::printf("    %-14s   %zu distinct: %s\n", "", s.distinctValues.size(), list.c_str());
    }
  }

  if (cloud.colorStatsValid) {
    static const char* kChannelNames[] = {"red", "green", "blue", "alpha"};
    std::printf("\n  \x1b[1mcolour channels\x1b[0m\n");
    bool greyscale = true;
    for (int ch = 0; ch < 3; ++ch) {
      const FieldStats& s = cloud.colorChannelStats[static_cast<std::size_t>(ch)];
      if (!s.valid()) continue;
      std::printf("    %-14s %-9s %-10s %14.0f %14.0f %14.2f %12.2f  %s\n", kChannelNames[ch],
                  "uint8", "color", s.min, s.max, s.mean, s.stddev,
                  sparkline(s.histogram, 24).c_str());
    }
    greyscale = cloud.colorIsGreyscale;
    if (greyscale)
      std::printf("    \x1b[33mR = G = B on %.1f%% of points: greyscale, probably "
                  "intensity stored as RGB\x1b[0m\n",
                  cloud.colorGreyscaleFraction * 100.0);
  }

  const LoadTiming& t = cloud.timing;
  std::printf("\n  \x1b[1mload\x1b[0m  %.0f ms total   (%.0f MB/s)\n", t.totalMs,
              t.throughputMBs(cloud.sourceBytes));
  std::printf("    map %.1f  header %.1f  decode %.1f  stats %.1f  (ms)\n", t.mapMs, t.headerMs,
              t.decodeMs, t.statsMs);
  std::printf("    %u worker threads\n", defaultWorkerCount());

  for (const std::string& w : result.warnings)
    std::printf("  \x1b[33mwarning\x1b[0m  %s\n", w.c_str());
  std::printf("\n");
}

void printChunkReport(const PointCloud& cloud, const std::vector<Chunk>& chunks,
                      const ChunkBuildStats& stats) {
  std::printf("  \x1b[1mchunking\x1b[0m  %.0f ms\n", stats.totalMs);
  std::printf("    partition %.1f  lod-order %.1f  reorder %.1f  bounds %.1f  (ms)\n",
              stats.partitionMs, stats.lodShuffleMs, stats.reorderMs, stats.boundsMs);
  std::printf("    chunks        %s\n", withThousands(stats.chunkCount).c_str());
  std::printf("    points/chunk  min %s  mean %.0f  max %s\n",
              withThousands(stats.minChunkPoints).c_str(), stats.meanChunkPoints,
              withThousands(stats.maxChunkPoints).c_str());

  // What the GPU will actually hold.
  const std::uint64_t n = cloud.size();
  const std::uint64_t posBytes = n * sizeof(QuantizedPoint);
  const std::uint64_t colBytes = cloud.hasColors() ? n * 4 : 0;
  const std::uint64_t scalarBytes = n * 2;
  const std::uint64_t chunkBytes = chunks.size() * sizeof(GpuChunk);
  std::printf("    GPU vertex    %s positions + %s colour + %s scalar + %s chunk meta\n",
              humanBytes(posBytes).c_str(), humanBytes(colBytes).c_str(),
              humanBytes(scalarBytes).c_str(), humanBytes(chunkBytes).c_str());
  std::printf("    GPU total     \x1b[1m%s\x1b[0m\n",
              humanBytes(posBytes + colBytes + scalarBytes + chunkBytes).c_str());

  // Quantization error: worst case is half a step along the longest axis of
  // the largest chunk.
  double worstStep = 0.0;
  double meanSpacing = 0.0;
  for (const Chunk& c : chunks) {
    const Vec3f e = c.bounds.size();
    worstStep = std::max<double>(worstStep, std::max(e.x, std::max(e.y, e.z)) / 65535.0);
    meanSpacing += c.spacing;
  }
  if (!chunks.empty()) meanSpacing /= static_cast<double>(chunks.size());
  std::printf("    quantization  %.3f mm worst-case error (half a step)\n", worstStep * 500.0);
  std::printf("    mean spacing  %.1f mm between neighbouring points\n", meanSpacing * 1000.0);

  std::vector<double> spacings;
  spacings.reserve(chunks.size());
  for (const Chunk& c : chunks) spacings.push_back(c.spacing);
  std::sort(spacings.begin(), spacings.end());
  auto pct = [&](double p) {
    return spacings.empty() ? 0.0 : spacings[std::min(spacings.size() - 1,
                                     static_cast<std::size_t>(p * spacings.size()))];
  };
  std::printf("    spacing pct   p5 %.0f  p50 %.0f  p95 %.0f  p99 %.0f  max %.0f mm\n",
              pct(0.05) * 1000, pct(0.50) * 1000, pct(0.95) * 1000, pct(0.99) * 1000,
              spacings.empty() ? 0.0 : spacings.back() * 1000);
  // The worst offenders drive splat size, so name them.
  // Chunks too small to be worth a draw command, and how much of the cloud
  // they actually hold.
  std::size_t tiny = 0;
  std::uint64_t tinyPoints = 0;
  for (const Chunk& c : chunks) {
    if (c.pointCount < 1024) { ++tiny; tinyPoints += c.pointCount; }
  }
  std::printf("    tiny chunks   %zu under 1024 points (%.1f%% of chunks, %.3f%% of points)\n",
              tiny, chunks.empty() ? 0.0 : 100.0 * static_cast<double>(tiny) / static_cast<double>(chunks.size()),
              n == 0 ? 0.0 : 100.0 * static_cast<double>(tinyPoints) / static_cast<double>(n));

  std::printf("    widest chunks:\n");
  std::vector<const Chunk*> byspace;
  for (const Chunk& c : chunks) byspace.push_back(&c);
  std::sort(byspace.begin(), byspace.end(),
            [](const Chunk* a, const Chunk* b) { return a->spacing > b->spacing; });
  for (std::size_t i = 0; i < std::min<std::size_t>(5, byspace.size()); ++i) {
    const Chunk* c = byspace[i];
    const Vec3f e = c->bounds.size();
    std::printf("      spacing %6.0f mm  %8u pts  box %.1f x %.1f x %.1f m\n",
                c->spacing * 1000.0, c->pointCount, e.x, e.y, e.z);
  }

  // Verify the invariant the renderer depends on.
  std::uint64_t covered = 0;
  bool contiguous = true;
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    covered += chunks[i].pointCount;
    if (i > 0 && chunks[i].firstPoint != chunks[i - 1].lastPoint()) contiguous = false;
  }
  std::printf("    coverage      %s (%s)\n",
              covered == n ? "\x1b[32mall points in exactly one chunk\x1b[0m" : "\x1b[31mMISMATCH\x1b[0m",
              contiguous ? "contiguous" : "\x1b[31mnot contiguous\x1b[0m");
  std::printf("\n");
}

// Header-only report: what the file declares and what opening it would cost.
// Deliberately prints before anything large is allocated, which is the whole
// point of having it.
void printEstimate(const CloudInfo& info) {
  std::printf("\n\x1b[1m%s\x1b[0m\n", info.formatName.c_str());
  std::printf("  file size     %s\n", humanBytes(info.fileBytes).c_str());
  std::printf("  points        %s\n", withThousands(info.pointCount).c_str());
  std::printf("  record        %zu B x %s declared fields\n", info.recordStride,
              withThousands(info.fields.size()).c_str());

  std::printf("  fields        ");
  for (std::size_t i = 0; i < info.fields.size(); ++i)
    std::printf("%s%s", i == 0 ? "" : " ", info.fields[i].name.c_str());
  std::printf("\n");

  if (info.boundsKnown) {
    const Vec3 size{info.maxBound.x - info.minBound.x, info.maxBound.y - info.minBound.y,
                    info.maxBound.z - info.minBound.z};
    std::printf("  extent        %.3f x %.3f x %.3f m  (from the header)\n", size.x, size.y,
                size.z);
  } else {
    std::printf("  extent        unknown until the points are read\n");
  }

  const MemoryEstimate& m = info.memory;
  std::printf("\n  \x1b[1mpredicted cost\x1b[0m\n");
  std::printf("    per point     %u B in RAM\n", m.bytesPerPoint);
  if (m.decodeBuffer > 0)
    std::printf("    decode buffer %s   (decompressed records, freed after load)\n",
                humanBytes(m.decodeBuffer).c_str());
  std::printf("    point cloud   %s\n", humanBytes(m.pointCloud).c_str());
  std::printf("    chunk scratch %s   (transient, during chunk building)\n",
              humanBytes(m.chunkScratch).c_str());
  std::printf("    \x1b[1mpeak RAM      %s\x1b[0m   (%s allowing for allocator overhead)\n",
              humanBytes(m.peakHost()).c_str(), humanBytes(m.peakHostPadded()).c_str());
  std::printf("    GPU vertex    %s\n", humanBytes(m.gpuVertex).c_str());

  const std::uint64_t avail = availableMemoryBytes();
  const std::uint64_t total = totalMemoryBytes();
  if (avail > 0) {
    std::printf("\n  host          %s available of %s\n", humanBytes(avail).c_str(),
                humanBytes(total).c_str());
    if (m.peakHostPadded() > avail)
      std::printf("  \x1b[31mverdict       will not fit: needs %s more than is available\x1b[0m\n",
                  humanBytes(m.peakHostPadded() - avail).c_str());
    else if (m.peakHostPadded() > avail / 2)
      std::printf("  \x1b[33mverdict       fits, but uses over half of available memory\x1b[0m\n");
    else
      std::printf("  \x1b[32mverdict       fits comfortably\x1b[0m\n");
  }

  for (const std::string& w : info.warnings)
    std::printf("  \x1b[33mwarning\x1b[0m       %s\n", w.c_str());
}

// Splits a comma-separated field list, e.g. "gps_time,user_data".
std::vector<std::string> splitFieldList(const std::string& text) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::size_t end = comma == std::string::npos ? text.size() : comma;
    if (end > start) out.push_back(text.substr(start, end - start));
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return out;
}

void usage() {
  std::printf(
      "usage: punto-probe [options] <file> [<file> ...]\n"
      "\n"
      "  --keep-invalid     retain points with NaN/Inf coordinates\n"
      "  --no-source-index  do not record each point's source row\n"
      "  --threads N        worker threads (default: all cores)\n"
      "  --bin-fields N     float32 fields per point for raw .bin (default 4)\n"
      "  --estimate         read only the header: report size and memory cost,\n"
      "                     then exit without loading any points\n"
      "  --max-points N     keep at most N points, evenly spread through the file\n"
      "  --drop-fields A,B  do not decode these scalar fields at all\n"
      "  --chunks           build the chunk/LOD tree and report it\n"
      "  --chunk-size N     max points per chunk (default 131072)\n"
      "  -v                 verbose timing\n");
}

}  // namespace

int main(int argc, char** argv) {
  LoadOptions options;
  ChunkBuildOptions chunkOptions;
  bool doChunks = false;
  bool estimateOnly = false;
  std::vector<std::string> paths;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--keep-invalid") options.dropInvalid = false;
    else if (arg == "--no-source-index") options.keepSourceIndex = false;
    else if (arg == "--threads" && i + 1 < argc) options.workers = static_cast<unsigned>(std::atoi(argv[++i]));
    else if (arg == "--bin-fields" && i + 1 < argc) options.rawBinFieldCount = std::atoi(argv[++i]);
    else if (arg == "--max-points" && i + 1 < argc)
      options.maxPoints = std::strtoull(argv[++i], nullptr, 10);
    else if (arg == "--drop-fields" && i + 1 < argc)
      options.dropFields = splitFieldList(argv[++i]);
    else if (arg == "--estimate") estimateOnly = true;
    else if (arg == "--chunks") doChunks = true;
    else if (arg == "--chunk-size" && i + 1 < argc) {
      chunkOptions.maxPointsPerChunk = static_cast<unsigned>(std::atoi(argv[++i]));
      doChunks = true;
    }
    else if (arg == "-v") setLogLevel(LogLevel::Debug);
    else if (arg == "-h" || arg == "--help") { usage(); return 0; }
    else if (!arg.empty() && arg[0] == '-') { std::printf("unknown option: %s\n", arg.c_str()); usage(); return 2; }
    else paths.push_back(arg);
  }

  if (paths.empty()) { usage(); return 2; }

  int failures = 0;
  for (const std::string& path : paths) {
    if (estimateOnly) {
      const CloudInfo info = probeCloud(path, options);
      if (!info.ok) {
        std::fprintf(stderr, "\x1b[31merror\x1b[0m  %s: %s\n", path.c_str(), info.error.c_str());
        ++failures;
        continue;
      }
      std::printf("\n\x1b[1m%s\x1b[0m", path.c_str());
      printEstimate(info);
      continue;
    }

    PointCloud cloud;
    const LoadResult result = loadPointCloud(path, options, cloud);
    if (!result.ok) {
      std::fprintf(stderr, "\x1b[31merror\x1b[0m  %s: %s\n", path.c_str(), result.error.c_str());
      ++failures;
      continue;
    }
    printReport(cloud, result);

    if (doChunks) {
      ChunkBuildStats stats;
      const std::vector<Chunk> chunks = buildChunks(cloud, chunkOptions, &stats);
      printChunkReport(cloud, chunks, stats);
    }
  }
  return failures == 0 ? 0 : 1;
}
