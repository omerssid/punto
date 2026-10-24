#include "scene/ram_chunk_source.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace pcv {
namespace {

constexpr float kQuantMax = 65535.0f;

// Maps a coordinate onto [0, 65535] within the chunk's extent along one axis.
inline std::uint16_t quantize(float value, float lo, float invExtent) {
  const float t = (value - lo) * invExtent;
  return static_cast<std::uint16_t>(std::clamp(t, 0.0f, 1.0f) * kQuantMax + 0.5f);
}

}  // namespace

RamChunkSource::RamChunkSource(std::shared_ptr<PointCloud> cloud, std::vector<Chunk> chunks)
    : cloud_(std::move(cloud)), chunks_(std::move(chunks)) {}

void RamChunkSource::readChunkPositions(std::size_t chunkIndex,
                                        std::span<QuantizedPoint> dst) const {
  const Chunk& chunk = chunks_[chunkIndex];
  const std::size_t count = std::min<std::size_t>(chunk.pointCount, dst.size());
  const Vec3f extent = chunk.bounds.size();

  // A chunk can be flat in an axis (a wall, a road surface). Collapsing that
  // axis to zero is correct: every point genuinely shares the coordinate.
  const Vec3f inv{extent.x > 0.0f ? 1.0f / extent.x : 0.0f,
                  extent.y > 0.0f ? 1.0f / extent.y : 0.0f,
                  extent.z > 0.0f ? 1.0f / extent.z : 0.0f};

  const Vec3f* src = cloud_->positions.data() + chunk.firstPoint;
  for (std::size_t i = 0; i < count; ++i) {
    const Vec3f& p = src[i];
    dst[i] = QuantizedPoint{quantize(p.x, chunk.bounds.lo.x, inv.x),
                            quantize(p.y, chunk.bounds.lo.y, inv.y),
                            quantize(p.z, chunk.bounds.lo.z, inv.z), 0};
  }
}

bool RamChunkSource::readChunkColors(std::size_t chunkIndex,
                                     std::span<std::uint32_t> dst) const {
  if (!cloud_->hasColors()) return false;
  const Chunk& chunk = chunks_[chunkIndex];
  const std::size_t count = std::min<std::size_t>(chunk.pointCount, dst.size());
  std::memcpy(dst.data(), cloud_->colors.data() + chunk.firstPoint,
              count * sizeof(std::uint32_t));
  return true;
}

bool RamChunkSource::readChunkScalars(std::size_t chunkIndex, int fieldIndex, double lo, double hi,
                                      std::span<std::uint16_t> dst) const {
  if (fieldIndex < 0 || fieldIndex >= static_cast<int>(cloud_->fields.size())) return false;
  const FieldInfo& field = cloud_->fields[static_cast<std::size_t>(fieldIndex)];

  const Chunk& chunk = chunks_[chunkIndex];
  const std::size_t count = std::min<std::size_t>(chunk.pointCount, dst.size());
  // A zero-width range would divide by zero; map everything to the bottom
  // of the ramp instead.
  const double span = hi - lo;
  const double scale = span > 0.0 ? kQuantMax / span : 0.0;

  auto store = [&](std::size_t i, double value) {
    const double t = (value - lo) * scale;
    dst[i] = static_cast<std::uint16_t>(std::clamp(t, 0.0, static_cast<double>(kQuantMax)));
  };

  if (field.role == FieldRole::Position) {
    // x/y/z have no scalar column; read straight from the position array.
    // Values are reported in world coordinates so they line up with the
    // ranges shown in the statistics panel.
    const int axis = field.name == "x" ? 0 : (field.name == "y" ? 1 : 2);
    const double offset = cloud_->origin[axis];
    const Vec3f* src = cloud_->positions.data() + chunk.firstPoint;
    for (std::size_t i = 0; i < count; ++i) store(i, static_cast<double>(src[i][axis]) + offset);
    return true;
  }

  if (field.columnIndex < 0) return false;  // packed colour: not a scalar ramp
  const ScalarColumn& column = cloud_->columns[static_cast<std::size_t>(field.columnIndex)];

  // visit() monomorphises the loop for the column's concrete type, so this
  // is a tight typed loop rather than a per-point switch.
  column.visit([&](auto span) {
    const auto* src = span.data() + chunk.firstPoint;
    for (std::size_t i = 0; i < count; ++i) store(i, static_cast<double>(src[i]));
  });
  return true;
}

}  // namespace pcv
