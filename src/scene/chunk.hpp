// Spatial chunking and the interface the renderer draws through.
//
// A chunk is a spatially compact run of points that the GPU treats as one
// unit: one frustum test, one LOD decision, one indirect draw. Points inside
// a chunk are permuted at build time into LOD order, so drawing the first N
// of them yields a uniform subsample of the whole chunk -- which is how level
// of detail costs zero extra memory here.
#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "core/math.hpp"
#include "io/point_cloud.hpp"

namespace pcv {

// GPU vertex: position quantized to the owning chunk's bounding box.
//
// Chunks span tens of metres, so 16 bits per axis resolves well under a
// millimetre -- far below LiDAR noise -- at 8 bytes/point including the
// padding lane that keeps the stride 4-byte aligned.
struct QuantizedPoint {
  std::uint16_t x, y, z, pad;
};
static_assert(sizeof(QuantizedPoint) == 8);

struct Chunk {
  Aabbf bounds;                  // local to PointCloud::origin
  std::uint32_t firstPoint = 0;  // offset into the cloud's arrays
  std::uint32_t pointCount = 0;
  // Approximate distance between neighbouring points, used to size splats
  // and to pick how many points a chunk needs at a given screen size.
  float spacing = 0.0f;

  std::uint32_t lastPoint() const { return firstPoint + pointCount; }
};

// Per-chunk record the culling compute shader reads. Laid out for std430:
// vec4s first so nothing straddles a 16-byte boundary.
struct alignas(16) GpuChunk {
  float boundsMin[4];  // xyz = min corner, w = spacing
  float boundsMax[4];  // xyz = max corner, w unused
  std::uint32_t firstPoint;
  std::uint32_t pointCount;
  std::uint32_t pad0;
  std::uint32_t pad1;
};
static_assert(sizeof(GpuChunk) == 48);

// Supplies chunk geometry and the vertex data behind it.
//
// The renderer only ever asks this interface for bytes, never for the
// PointCloud itself. That is what lets a disk-backed, streaming
// implementation replace RamChunkSource later without the renderer, the LOD
// selector, the culling pass or the UI changing at all.
class ChunkSource {
 public:
  virtual ~ChunkSource() = default;

  virtual std::span<const Chunk> chunks() const = 0;
  virtual std::uint64_t totalPoints() const = 0;
  virtual Aabbf bounds() const = 0;
  // World offset the local coordinates are relative to.
  virtual Vec3 origin() const = 0;

  // Writes `chunk.pointCount` quantized vertices for `chunkIndex` into `dst`.
  virtual void readChunkPositions(std::size_t chunkIndex,
                                  std::span<QuantizedPoint> dst) const = 0;

  // Writes packed RGBA8 colour, or returns false when the cloud has none.
  virtual bool readChunkColors(std::size_t chunkIndex, std::span<std::uint32_t> dst) const = 0;

  // Writes the named field's values normalised to uint16 over [lo, hi].
  // Returns false if the field index is not a valid scalar source.
  virtual bool readChunkScalars(std::size_t chunkIndex, int fieldIndex, double lo, double hi,
                                std::span<std::uint16_t> dst) const = 0;

  std::size_t chunkCount() const { return chunks().size(); }
};

struct ChunkBuildOptions {
  // Upper bound on points per chunk. Smaller chunks cull and LOD more
  // precisely but add per-chunk overhead in the compute pass and more
  // indirect draw commands.
  std::uint32_t maxPointsPerChunk = 131072;
  // Guards against unbounded recursion when many points share a position.
  int maxDepth = 16;
  unsigned workers = 0;
};

struct ChunkBuildStats {
  double partitionMs = 0.0;
  double reorderMs = 0.0;
  double lodShuffleMs = 0.0;
  double boundsMs = 0.0;
  double totalMs = 0.0;
  std::size_t chunkCount = 0;
  std::uint32_t minChunkPoints = 0;
  std::uint32_t maxChunkPoints = 0;
  double meanChunkPoints = 0.0;
};

// Partitions `cloud` into chunks, reordering the cloud's arrays in place so
// each chunk occupies a contiguous range, then permutes within each chunk
// into LOD order. PointCloud::sourceIndex preserves the original file rows.
std::vector<Chunk> buildChunks(PointCloud& cloud, const ChunkBuildOptions& options,
                               ChunkBuildStats* stats = nullptr);

}  // namespace pcv
