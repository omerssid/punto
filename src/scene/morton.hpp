// Morton (Z-order) curve helpers.
//
// Interleaving the bits of a point's grid coordinates maps 3D position onto
// a single integer that preserves locality: points close in space are close
// in Morton order. Sorting by it turns octree construction into a problem of
// finding contiguous ranges, because every node at level L is exactly the
// run of points sharing the top 3L bits of their code.
#pragma once

#include <algorithm>
#include <cstdint>

#include "core/math.hpp"

namespace pcv {

// Bits of grid resolution per axis. Ten gives a 1024^3 grid, so the finest
// representable cell is 1/1024 of the cloud's longest side -- far beyond
// the five to seven levels of subdivision chunking actually uses.
//
// Ten is also what lets the code and a 32-bit point index share one 64-bit
// word: 30 + 32 = 62 bits. Halving the element size halves the memory
// traffic of the radix sort, which is the whole point of the exercise.
inline constexpr int kMortonBits = 10;
inline constexpr std::uint32_t kMortonGridMax = (1u << kMortonBits) - 1u;
inline constexpr int kMortonCodeBits = 3 * kMortonBits;
inline constexpr int kMortonMaxLevel = kMortonBits;

// Spreads the low 10 bits of x so two zero bits sit between each, which is
// what lets three coordinates interleave without colliding.
inline std::uint32_t spreadBits(std::uint32_t x) {
  x &= 0x000003FFu;
  x = (x | (x << 16)) & 0x030000FFu;
  x = (x | (x << 8)) & 0x0300F00Fu;
  x = (x | (x << 4)) & 0x030C30C3u;
  x = (x | (x << 2)) & 0x09249249u;
  return x;
}

inline std::uint32_t mortonCode(std::uint32_t x, std::uint32_t y, std::uint32_t z) {
  return spreadBits(x) | (spreadBits(y) << 1) | (spreadBits(z) << 2);
}

// Maps world positions onto the integer grid the codes are computed over.
//
// The grid is cubic -- sized by the longest axis -- so a cell is a cube and
// Morton order means the same thing along every axis. A grid stretched to
// the bounding box would make cells flat for a cloud like an HD map, and
// the resulting "octree" nodes would be flat too.
struct MortonGrid {
  Vec3f origin{0.0f, 0.0f, 0.0f};
  float scale = 0.0f;   // grid cells per metre
  float cellSize = 0.0f;

  static MortonGrid forBounds(const Aabbf& bounds) {
    MortonGrid grid;
    if (!bounds.valid()) return grid;
    grid.origin = bounds.lo;
    const float extent = std::max(maxComponent(bounds.size()), 1e-6f);
    grid.scale = static_cast<float>(kMortonGridMax) / extent;
    grid.cellSize = extent / static_cast<float>(kMortonGridMax);
    return grid;
  }

  std::uint32_t codeFor(const Vec3f& p) const {
    const auto quantize = [&](float v, float lo) {
      const float t = (v - lo) * scale;
      return static_cast<std::uint32_t>(
          std::clamp(t, 0.0f, static_cast<float>(kMortonGridMax)));
    };
    return mortonCode(quantize(p.x, origin.x), quantize(p.y, origin.y),
                      quantize(p.z, origin.z));
  }

  // Side length of a node at `level`, in metres. Level 0 is the whole grid.
  float nodeExtent(int level) const {
    return cellSize * static_cast<float>(kMortonGridMax + 1) /
           static_cast<float>(1u << level);
  }
};

// One sortable element: the Morton code in the high bits so a sort on the
// code alone orders the array, with the point's original index carried
// along in the low 32.
inline constexpr int kMortonKeyShift = 32;

inline std::uint64_t makeMortonKey(std::uint32_t code, std::uint32_t index) {
  return (static_cast<std::uint64_t>(code) << kMortonKeyShift) | index;
}

inline std::uint32_t keyCode(std::uint64_t key) {
  return static_cast<std::uint32_t>(key >> kMortonKeyShift);
}

inline std::uint32_t keyIndex(std::uint64_t key) {
  return static_cast<std::uint32_t>(key & 0xFFFFFFFFu);
}

// The octree node a key belongs to at `level`: its top 3*level code bits.
inline std::uint64_t keyPrefix(std::uint64_t key, int level) {
  const int drop = kMortonKeyShift + (kMortonCodeBits - 3 * level);
  return key >> drop;
}

}  // namespace pcv
