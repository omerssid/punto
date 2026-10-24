// The in-memory representation every loader produces and every consumer reads.
//
// Layout is columnar (struct-of-arrays): positions in one array, each other
// attribute in its own ScalarColumn. That makes per-field statistics touch
// only the bytes of that field, lets a newly selected colour source upload to
// the GPU as a single contiguous copy, and keeps narrow fields narrow.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "core/math.hpp"
#include "io/field.hpp"

namespace pcv {

// Where the load spent its time. Surfaced in the UI so a slow open is
// immediately attributable rather than mysterious.
struct LoadTiming {
  double mapMs = 0.0;      // opening and mapping the file
  double headerMs = 0.0;   // parsing the header
  double decodeMs = 0.0;   // the parallel point decode
  double statsMs = 0.0;    // per-field statistics
  double totalMs = 0.0;

  double throughputMBs(std::uint64_t bytes) const {
    return totalMs > 0.0 ? (static_cast<double>(bytes) / 1e6) / (totalMs / 1e3) : 0.0;
  }
};

class PointCloud {
 public:
  // ------------------------------------------------------------ geometry

  // Positions are stored LOCAL to `origin`, as float32.
  //
  // HD maps and SLAM output carry UTM-like absolute coordinates where float32
  // resolves to roughly half a metre. Subtracting a double-precision origin at
  // parse time brings every coordinate near zero, where float32 gives sub-
  // millimetre resolution across a multi-kilometre map. Absolute coordinates
  // are recovered on demand via worldPosition(); `origin` itself stays double.
  Vec3 origin{0.0, 0.0, 0.0};
  std::vector<Vec3f> positions;

  Aabbf boundsLocal;  // over `positions`
  Aabb boundsWorld;   // boundsLocal shifted by origin

  // ---------------------------------------------------------- attributes

  // Packed R8G8B8A8, little-endian in a uint32. Empty when the source had
  // no colour, in which case the viewer falls back to a scalar colour ramp.
  std::vector<std::uint32_t> colors;

  // One entry per non-position, non-packed-colour attribute.
  std::vector<ScalarColumn> columns;

  // Row number each point occupied in the source file.
  //
  // Invalid points are dropped at load and LOD reorders points within chunks,
  // so a point's slot in `positions` says nothing about where it came from.
  // Keeping the original row lets the inspector report a number that can be
  // cross-referenced against the raw file or another tool. Empty if the
  // loader was told not to track it.
  std::vector<std::uint32_t> sourceIndex;

  // Describes every field the source declared, including x/y/z and packed
  // rgb. Entries whose data was folded into `positions` or `colors` carry
  // columnIndex == -1.
  std::vector<FieldInfo> fields;

  // -------------------------------------------------------------- source

  std::string sourcePath;
  std::string formatName;
  std::uint64_t sourceBytes = 0;
  LoadTiming timing;

  // Per-channel statistics over `colors`, in R,G,B,A order.
  //
  // Packed rgb/rgba fields fold into `colors` and so have no ScalarColumn of
  // their own. Deriving channel stats keeps them visible in the inspector --
  // it is how you tell a genuinely coloured cloud from one that is
  // accidentally greyscale or uniformly white.
  std::array<FieldStats, 4> colorChannelStats{};
  bool colorStatsValid = false;

  // Points rejected during load for having non-finite coordinates.
  std::uint64_t invalidPointsDropped = 0;

  // True when the source declared an organised cloud (HEIGHT > 1).
  bool organized = false;
  std::uint32_t width = 0;
  std::uint32_t height = 0;

  // ------------------------------------------------------------ accessors

  std::size_t size() const { return positions.size(); }
  bool empty() const { return positions.empty(); }
  bool hasColors() const { return colors.size() == positions.size() && !colors.empty(); }
  bool hasSourceIndex() const { return sourceIndex.size() == positions.size(); }

  // Row in the source file, or the array slot if tracking was disabled.
  std::uint64_t sourceRow(std::size_t i) const {
    return hasSourceIndex() ? sourceIndex[i] : i;
  }

  Vec3 worldPosition(std::size_t i) const {
    const Vec3f& p = positions[i];
    return Vec3{origin.x + static_cast<double>(p.x), origin.y + static_cast<double>(p.y),
                origin.z + static_cast<double>(p.z)};
  }

  const FieldInfo* findField(std::string_view name) const;
  // Fields the UI can colour by: every scalar column, plus the synthetic
  // x/y/z entries. Returns indices into `fields`.
  std::vector<int> colorableFieldIndices() const;

  // Approximate resident size, for the memory readout.
  std::uint64_t memoryBytes() const;

  // Recomputes boundsLocal/boundsWorld from `positions`, in parallel.
  void recomputeBounds();
  // Fills FieldStats for every field, including the synthetic x/y/z entries.
  void computeAllStats();

  void clear();
};

}  // namespace pcv
