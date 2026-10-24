// Typed, type-erased storage for a single point attribute column.
//
// Clouds in the wild declare arbitrary fields -- `x y z intensity ring t`,
// `x y z rgb label`, normals, curvature, GPS time. Rather than forcing them
// into a fixed point struct the way PCL's PointXYZI/PointXYZRGB templates do,
// every non-position field becomes a ScalarColumn that keeps the exact width
// and signedness the file declared. A uint8 `ring` costs one byte per point,
// and a float64 timestamp keeps every bit, so the inspector can report values
// that match the file exactly.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace punto {

enum class FieldType : std::uint8_t { I8, I16, I32, I64, U8, U16, U32, U64, F32, F64 };

std::size_t fieldTypeSize(FieldType t);
const char* fieldTypeName(FieldType t);
bool fieldTypeIsFloat(FieldType t);
bool fieldTypeIsSigned(FieldType t);
// Maps a PCD-style (TYPE char, SIZE) pair, e.g. ('F', 4), onto a FieldType.
bool fieldTypeFromPcd(char typeChar, std::size_t size, FieldType& out);

// What a field *means*, inferred from its name. Drives default colouring,
// axis assignment, and which fields the UI offers as a colour source.
enum class FieldRole : std::uint8_t {
  Position,   // x, y, z -- extracted into PointCloud::positions, not a column
  Intensity,
  Color,      // packed rgb/rgba, or separate r/g/b channels
  Normal,
  Label,      // semantic/instance id; discrete
  Ring,       // LiDAR beam index; discrete
  Time,
  Other,
};

FieldRole inferFieldRole(std::string_view name);
const char* fieldRoleName(FieldRole r);

// Summary statistics for one column, computed once during load.
struct FieldStats {
  double min = 0.0;
  double max = 0.0;
  double mean = 0.0;
  double stddev = 0.0;
  std::uint64_t finiteCount = 0;   // values that are neither NaN nor infinite
  std::uint64_t nonFiniteCount = 0;

  // 256-bin histogram over [min, max]. Drives the colour-ramp widget and
  // makes outliers visible at a glance.
  std::vector<std::uint32_t> histogram;

  // Populated only for narrow integer columns (ring, label) where a small
  // distinct-value count is genuinely informative.
  bool discrete = false;
  std::vector<double> distinctValues;  // capped; empty if too many

  bool valid() const { return finiteCount > 0; }
};

// One column of per-point scalar data, stored in its declared native type.
class ScalarColumn {
 public:
  ScalarColumn() = default;
  ScalarColumn(FieldType type, std::size_t count) { reset(type, count); }

  void reset(FieldType type, std::size_t count);

  FieldType type() const { return type_; }
  std::size_t count() const { return count_; }
  std::size_t elementSize() const { return fieldTypeSize(type_); }
  std::size_t byteSize() const { return bytes_.size(); }

  std::byte* data() { return bytes_.data(); }
  const std::byte* data() const { return bytes_.data(); }

  // Reads element i widened to double. Used by the inspector, stats and
  // GPU quantization -- never on a per-point hot loop where the type is
  // already known (use visit() there instead).
  double at(std::size_t i) const;

  template <typename T>
  T* typedData() { return reinterpret_cast<T*>(bytes_.data()); }
  template <typename T>
  const T* typedData() const { return reinterpret_cast<const T*>(bytes_.data()); }

  // Calls fn(std::span<const T>) with the column's concrete type, so callers
  // write one template body and get a monomorphised loop per type.
  template <typename Fn>
  decltype(auto) visit(Fn&& fn) const;

 private:
  FieldType type_ = FieldType::F32;
  std::size_t count_ = 0;
  std::vector<std::byte> bytes_;
};

// Describes a column: what it is called, what it holds, what it means.
struct FieldInfo {
  std::string name;
  FieldType type = FieldType::F32;
  FieldRole role = FieldRole::Other;
  // Index into PointCloud::columns, or -1 for fields consumed during load
  // (x/y/z fold into positions; packed rgb folds into the colour array).
  int columnIndex = -1;
  FieldStats stats;
};

template <typename Fn>
decltype(auto) ScalarColumn::visit(Fn&& fn) const {
  const std::size_t n = count_;
  switch (type_) {
    case FieldType::I8:  return fn(std::span<const std::int8_t>(typedData<std::int8_t>(), n));
    case FieldType::I16: return fn(std::span<const std::int16_t>(typedData<std::int16_t>(), n));
    case FieldType::I32: return fn(std::span<const std::int32_t>(typedData<std::int32_t>(), n));
    case FieldType::I64: return fn(std::span<const std::int64_t>(typedData<std::int64_t>(), n));
    case FieldType::U8:  return fn(std::span<const std::uint8_t>(typedData<std::uint8_t>(), n));
    case FieldType::U16: return fn(std::span<const std::uint16_t>(typedData<std::uint16_t>(), n));
    case FieldType::U32: return fn(std::span<const std::uint32_t>(typedData<std::uint32_t>(), n));
    case FieldType::U64: return fn(std::span<const std::uint64_t>(typedData<std::uint64_t>(), n));
    case FieldType::F32: return fn(std::span<const float>(typedData<float>(), n));
    case FieldType::F64: return fn(std::span<const double>(typedData<double>(), n));
  }
  return fn(std::span<const float>(typedData<float>(), n));
}

// Computes stats (and histogram) for a column, in parallel across workers.
FieldStats computeFieldStats(const ScalarColumn& column, FieldRole role);

}  // namespace punto
