// Shared machinery for turning declared source fields into a PointCloud.
//
// PCD binary, PLY binary and raw KITTI .bin all store points as fixed-stride
// array-of-structs records that differ only in how the fields are described.
// Planning (which source field becomes what), the AoS->SoA decode loop,
// origin selection and invalid-point compaction are therefore written once
// here and reused by every loader.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

#include "core/math.hpp"
#include "io/field.hpp"
#include "io/loader.hpp"
#include "io/point_cloud.hpp"

namespace pcv {

// One field as the source file declares it.
struct SourceField {
  std::string name;
  FieldType type = FieldType::F32;
  std::size_t count = 1;       // PCD COUNT; >1 expands into count columns
  std::size_t byteOffset = 0;  // within the AoS record; filled by layoutFields
};

// Where a decoded value ends up.
enum class FieldDest : std::uint8_t {
  PosX, PosY, PosZ,
  PackedRgb,    // 0x00RRGGBB in a float or uint32 (the PCL convention)
  PackedRgba,   // 0xAARRGGBB in a uint32
  ColorR, ColorG, ColorB, ColorA,
  Column,
  Skip,
};

struct DecodePlan {
  struct Entry {
    FieldDest dest = FieldDest::Skip;
    FieldType type = FieldType::F32;
    // Point i's value lives at base + srcOffset + i * srcStride.
    //
    // Carrying a per-entry stride lets one decode loop serve both layouts:
    // interleaved records set every stride to the record size, while PCD's
    // binary_compressed payload -- which is stored field-block by field-block
    // -- sets each entry's stride to that field's own element size and its
    // offset to the start of its block.
    std::size_t srcOffset = 0;
    std::size_t srcStride = 0;
    int column = -1;            // index into PointCloud::columns
    // Separate colour channels declared as floats are taken to be 0..1;
    // integer channels are taken to be 0..255.
    bool colorIsNormalizedFloat = false;

    // Affine transform applied to positions and colour channels as they are
    // read: value = raw * scale + offset. LAS needs both -- it stores
    // coordinates as scaled integers, and its colour channels are 16-bit
    // where ours are 8. Identity by default, and never applied to scalar
    // columns, which keep the file's exact bytes.
    double scale = 1.0;
    double offset = 0.0;

    // Byte-swap the raw value before interpreting it. Needed only for
    // PLY's binary_big_endian variant.
    bool byteSwap = false;
  };

  std::vector<Entry> entries;
  std::size_t recordStride = 0;
  std::size_t tokensPerRecord = 0;  // ASCII
  bool hasPos[3] = {false, false, false};
  bool producesColor = false;

  bool hasAllPositions() const { return hasPos[0] && hasPos[1] && hasPos[2]; }
};

// Assigns byteOffset to each field, in declaration order, and returns the
// total record stride.
std::size_t layoutFields(std::vector<SourceField>& fields);

// Points every entry at an interleaved record layout of the given stride.
void setInterleavedStride(DecodePlan& plan, std::size_t recordStride);

// Points entries at a field-block (struct-of-arrays) payload holding
// `pointCount` points, as PCD binary_compressed uses. Field blocks appear in
// declaration order; each holds every point's value for one field, and a
// multi-COUNT field keeps its components adjacent within each point's slot.
// `fields` must be the same list buildDecodePlan was given.
void setFieldBlockStrides(DecodePlan& plan, const std::vector<SourceField>& fields,
                          std::size_t pointCount);

// Builds the decode plan and populates cloud.fields / cloud.columns metadata.
// Does not allocate per-point storage; call allocateCloud for that.
DecodePlan buildDecodePlan(const std::vector<SourceField>& fields, PointCloud& cloud,
                           std::vector<std::string>& warnings);

// Sizes every per-point array in `cloud` for `n` points.
void allocateCloud(PointCloud& cloud, const DecodePlan& plan, std::size_t n,
                   const LoadOptions& options);

// Chooses a double-precision origin to rebase positions onto, by sampling
// records across the file. Positions are stored relative to this so float32
// keeps sub-millimetre resolution even for UTM-scale coordinates.
Vec3 chooseOrigin(const DecodePlan& plan, const std::byte* base, std::size_t pointCount);

// Decodes points [first, last) into `cloud`, writing to the same indices.
// Returns the number of points found to have non-finite coordinates.
std::uint64_t decodeRange(const DecodePlan& plan, const std::byte* base, std::size_t first,
                          std::size_t last, const Vec3& origin, bool keepSourceIndex,
                          PointCloud& cloud);

// Runs decodeRange across workers, polling `stop`. Returns false if the load
// was cancelled, in which case `cloud` holds partial data and must be dropped.
bool decodeParallel(const DecodePlan& plan, const std::byte* base, std::size_t pointCount,
                    const Vec3& origin, const LoadOptions& options, std::stop_token stop,
                    PointCloud& cloud, std::uint64_t& invalidCount);

// Removes points whose positions are non-finite, keeping every parallel array
// consistent. No-op (and no copying) when `invalidCount` is zero.
void compactInvalidPoints(PointCloud& cloud, std::uint64_t invalidCount);

// True when every coordinate of point i is finite.
bool positionFinite(const Vec3f& p);

}  // namespace pcv
