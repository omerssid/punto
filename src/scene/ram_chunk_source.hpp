// In-core ChunkSource: the whole cloud is resident in RAM.
//
// This is phase one. A DiskChunkSource that streams nodes from a disk octree
// would implement the same interface; because the renderer only ever calls
// through ChunkSource, nothing above this line would need to change.
#pragma once

#include <memory>
#include <vector>

#include "io/point_cloud.hpp"
#include "scene/chunk.hpp"

namespace punto {

class RamChunkSource final : public ChunkSource {
 public:
  // Takes the cloud by shared_ptr because the UI reads the same instance for
  // the inspector and statistics panels while the renderer uploads from it.
  RamChunkSource(std::shared_ptr<PointCloud> cloud, std::vector<Chunk> chunks);

  std::span<const Chunk> chunks() const override { return chunks_; }
  std::uint64_t totalPoints() const override { return cloud_->size(); }
  Aabbf bounds() const override { return cloud_->boundsLocal; }
  Vec3 origin() const override { return cloud_->origin; }

  void readChunkPositions(std::size_t chunkIndex, std::span<QuantizedPoint> dst) const override;
  bool readChunkColors(std::size_t chunkIndex, std::span<std::uint32_t> dst) const override;
  bool readChunkScalars(std::size_t chunkIndex, int fieldIndex, double lo, double hi,
                        std::span<std::uint16_t> dst) const override;

  const PointCloud& cloud() const { return *cloud_; }
  const std::shared_ptr<PointCloud>& cloudPtr() const { return cloud_; }

 private:
  std::shared_ptr<PointCloud> cloud_;
  std::vector<Chunk> chunks_;
};

}  // namespace punto
