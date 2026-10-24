// Chunk builder tests.
//
// The reorder step moves positions, colours, source rows and every scalar
// column through the same permutation. If any one of them fell out of step
// the result would be a cloud whose points carry other points' attributes --
// silently wrong, and invisible in a screenshot. Several tests below exist
// specifically to catch that.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "io/loader.hpp"
#include "scene/chunk.hpp"
#include "scene/ram_chunk_source.hpp"

namespace {

int g_failures = 0;
int g_checks = 0;

void fail(const char* file, int line, const std::string& what) {
  ++g_failures;
  std::printf("  \x1b[31mFAIL\x1b[0m %s:%d  %s\n", file, line, what.c_str());
}

#define CHECK(cond)                                            \
  do {                                                         \
    ++g_checks;                                                \
    if (!(cond)) fail(__FILE__, __LINE__, "expected: " #cond); \
  } while (0)

#define CHECK_EQ(a, b)                                                     \
  do {                                                                     \
    ++g_checks;                                                            \
    const auto va_ = (a);                                                  \
    const auto vb_ = (b);                                                  \
    if (!(va_ == vb_))                                                     \
      fail(__FILE__, __LINE__, std::string(#a " == " #b "  (") +           \
                                   std::to_string(va_) + " vs " +          \
                                   std::to_string(vb_) + ")");             \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                              \
  do {                                                                     \
    ++g_checks;                                                            \
    const double va_ = static_cast<double>(a);                             \
    const double vb_ = static_cast<double>(b);                             \
    if (!(std::fabs(va_ - vb_) <= (eps)))                                  \
      fail(__FILE__, __LINE__, std::string(#a " ~= " #b "  (") +           \
                                   std::to_string(va_) + " vs " +          \
                                   std::to_string(vb_) + ")");             \
  } while (0)

void beginTest(const char* name) { std::printf("\x1b[1m%s\x1b[0m\n", name); }

using namespace pcv;

std::filesystem::path fixtureDir() {
  if (const char* env = std::getenv("PCV_FIXTURES")) return env;
  return std::filesystem::path(__FILE__).parent_path() / "fixtures";
}

PointCloud loadFixture(const char* name) {
  PointCloud cloud;
  const LoadResult r = loadPointCloud((fixtureDir() / name).string(), {}, cloud);
  if (!r.ok) fail(__FILE__, __LINE__, std::string("load failed: ") + r.error);
  return cloud;
}

ChunkBuildOptions smallChunks(std::uint32_t maxPoints = 2048) {
  ChunkBuildOptions o;
  o.maxPointsPerChunk = maxPoints;
  return o;
}

void testCoverageAndContiguity() {
  beginTest("every point lands in exactly one chunk, chunks tile the array");
  PointCloud cloud = loadFixture("helix_binary.pcd");
  if (cloud.empty()) return;
  const std::size_t n = cloud.size();

  const std::vector<Chunk> chunks = buildChunks(cloud, smallChunks());
  CHECK(!chunks.empty());

  std::uint64_t covered = 0;
  std::uint32_t expectedStart = 0;
  for (const Chunk& c : chunks) {
    CHECK_EQ(c.firstPoint, expectedStart);
    CHECK(c.pointCount > 0);
    expectedStart = c.lastPoint();
    covered += c.pointCount;
  }
  CHECK_EQ(covered, static_cast<std::uint64_t>(n));
  CHECK_EQ(static_cast<std::size_t>(expectedStart), n);
}

void testPointsLieInsideTheirChunkBounds() {
  beginTest("every point is inside its own chunk's bounding box");
  PointCloud cloud = loadFixture("helix_binary.pcd");
  if (cloud.empty()) return;
  const std::vector<Chunk> chunks = buildChunks(cloud, smallChunks());

  std::size_t outside = 0;
  for (const Chunk& c : chunks) {
    for (std::uint32_t i = c.firstPoint; i < c.lastPoint(); ++i) {
      const Vec3f& p = cloud.positions[i];
      if (p.x < c.bounds.lo.x || p.x > c.bounds.hi.x || p.y < c.bounds.lo.y ||
          p.y > c.bounds.hi.y || p.z < c.bounds.lo.z || p.z > c.bounds.hi.z)
        ++outside;
    }
  }
  CHECK_EQ(outside, std::size_t{0});
}

// The critical one: attributes must still belong to the points they came in
// with. Keyed on the preserved source row, every attribute is checked
// against the value that row had before chunking reordered anything.
void testAttributesStayWithTheirPoints() {
  beginTest("reordering keeps positions, colours and columns in step");
  PointCloud before = loadFixture("helix_binary.pcd");
  if (before.empty()) return;

  // Snapshot by source row, from the pre-chunking cloud.
  struct Snapshot { Vec3f pos; double intensity; double ring; double label; double t; };
  auto columnOf = [](const PointCloud& c, const char* name) -> const ScalarColumn* {
    const FieldInfo* f = c.findField(name);
    if (f == nullptr || f->columnIndex < 0) return nullptr;
    return &c.columns[static_cast<std::size_t>(f->columnIndex)];
  };

  const ScalarColumn* bIntensity = columnOf(before, "intensity");
  const ScalarColumn* bRing = columnOf(before, "ring");
  const ScalarColumn* bLabel = columnOf(before, "label");
  const ScalarColumn* bT = columnOf(before, "t");
  CHECK(bIntensity && bRing && bLabel && bT);
  if (!(bIntensity && bRing && bLabel && bT)) return;

  std::unordered_map<std::uint32_t, Snapshot> expected;
  expected.reserve(before.size());
  for (std::size_t i = 0; i < before.size(); ++i)
    expected[before.sourceIndex[i]] =
        Snapshot{before.positions[i], bIntensity->at(i), bRing->at(i), bLabel->at(i), bT->at(i)};

  PointCloud after = before;
  buildChunks(after, smallChunks());

  const ScalarColumn* aIntensity = columnOf(after, "intensity");
  const ScalarColumn* aRing = columnOf(after, "ring");
  const ScalarColumn* aLabel = columnOf(after, "label");
  const ScalarColumn* aT = columnOf(after, "t");
  CHECK(aIntensity && aRing && aLabel && aT);
  if (!(aIntensity && aRing && aLabel && aT)) return;

  CHECK_EQ(after.size(), before.size());
  std::size_t mismatched = 0;
  for (std::size_t i = 0; i < after.size(); ++i) {
    const auto it = expected.find(after.sourceIndex[i]);
    if (it == expected.end()) { ++mismatched; continue; }
    const Snapshot& e = it->second;
    if (!(after.positions[i] == e.pos) || aIntensity->at(i) != e.intensity ||
        aRing->at(i) != e.ring || aLabel->at(i) != e.label || aT->at(i) != e.t)
      ++mismatched;
  }
  CHECK_EQ(mismatched, std::size_t{0});

  // Source rows must still be a permutation of the originals: no point
  // duplicated, none lost.
  std::vector<std::uint32_t> rows = after.sourceIndex;
  std::sort(rows.begin(), rows.end());
  CHECK(std::adjacent_find(rows.begin(), rows.end()) == rows.end());
  CHECK_EQ(rows.front(), std::uint32_t{0});
  CHECK_EQ(rows.back(), static_cast<std::uint32_t>(after.size() - 1));
}

// The property the whole LOD scheme rests on: any prefix of a chunk must be
// a representative sample of that chunk, not a corner of it.
void testPrefixIsARepresentativeSample() {
  beginTest("any prefix of a chunk samples the whole chunk (the LOD property)");
  PointCloud cloud = loadFixture("helix_binary.pcd");
  if (cloud.empty()) return;
  const std::vector<Chunk> chunks = buildChunks(cloud, smallChunks(1024));

  // Pick the largest chunk so the statistics are meaningful.
  const Chunk* biggest = nullptr;
  for (const Chunk& c : chunks)
    if (biggest == nullptr || c.pointCount > biggest->pointCount) biggest = &c;
  CHECK(biggest != nullptr);
  if (biggest == nullptr || biggest->pointCount < 256) return;

  auto centroidOf = [&](std::uint32_t first, std::uint32_t count) {
    Vec3 sum{0, 0, 0};
    for (std::uint32_t i = first; i < first + count; ++i)
      sum += Vec3(cloud.positions[i]);
    return sum / static_cast<double>(count);
  };

  const Vec3 full = centroidOf(biggest->firstPoint, biggest->pointCount);
  const double extent = static_cast<double>(maxComponent(biggest->bounds.size()));

  // A prefix of a shuffled chunk is a uniform random sample, so its centroid
  // converges on the chunk centroid. With m samples the standard error is
  // about extent / sqrt(m); allow a generous multiple of that. An unshuffled
  // chunk -- points still in spatial order -- would fail badly here, which
  // is exactly the regression this guards against.
  for (std::uint32_t fraction : {4u, 8u, 16u}) {
    const std::uint32_t m = biggest->pointCount / fraction;
    if (m < 32) continue;
    const Vec3 prefix = centroidOf(biggest->firstPoint, m);
    const double err = length(prefix - full);
    const double tolerance = 4.0 * extent / std::sqrt(static_cast<double>(m));
    if (err > tolerance)
      std::printf("    prefix 1/%u: centroid off by %.4f (tolerance %.4f)\n", fraction, err,
                  tolerance);
    CHECK(err <= tolerance);
  }
}

void testDeterminism() {
  beginTest("chunking is deterministic across runs");
  PointCloud a = loadFixture("helix_binary.pcd");
  PointCloud b = loadFixture("helix_binary.pcd");
  if (a.empty()) return;

  const std::vector<Chunk> ca = buildChunks(a, smallChunks());
  const std::vector<Chunk> cb = buildChunks(b, smallChunks());

  CHECK_EQ(ca.size(), cb.size());
  if (ca.size() != cb.size()) return;
  for (std::size_t i = 0; i < ca.size(); ++i) {
    CHECK_EQ(ca[i].firstPoint, cb[i].firstPoint);
    CHECK_EQ(ca[i].pointCount, cb[i].pointCount);
  }
  std::size_t diff = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (!(a.positions[i] == b.positions[i]) || a.sourceIndex[i] != b.sourceIndex[i]) ++diff;
  CHECK_EQ(diff, std::size_t{0});
}

void testQuantizationRoundTrip() {
  beginTest("16-bit quantization round-trips within half a step");
  auto cloud = std::make_shared<PointCloud>(loadFixture("helix_binary.pcd"));
  if (cloud->empty()) return;
  std::vector<Chunk> chunks = buildChunks(*cloud, smallChunks());
  const RamChunkSource source(cloud, chunks);

  double worstError = 0.0;
  double worstAllowed = 0.0;
  std::vector<QuantizedPoint> buffer;

  for (std::size_t ci = 0; ci < chunks.size(); ++ci) {
    const Chunk& chunk = chunks[ci];
    buffer.resize(chunk.pointCount);
    source.readChunkPositions(ci, buffer);

    const Vec3f extent = chunk.bounds.size();
    for (std::uint32_t i = 0; i < chunk.pointCount; ++i) {
      const Vec3f& original = cloud->positions[chunk.firstPoint + i];
      const QuantizedPoint& q = buffer[i];
      // Mirrors the dequantize the vertex shader performs.
      const Vec3f restored{
          chunk.bounds.lo.x + static_cast<float>(q.x) / 65535.0f * extent.x,
          chunk.bounds.lo.y + static_cast<float>(q.y) / 65535.0f * extent.y,
          chunk.bounds.lo.z + static_cast<float>(q.z) / 65535.0f * extent.z};
      for (int axis = 0; axis < 3; ++axis) {
        const double err = std::fabs(static_cast<double>(restored[axis]) -
                                     static_cast<double>(original[axis]));
        // Half a quantization step, plus a float32 rounding allowance.
        const double allowed = extent[axis] / 65535.0 * 0.5 + 1e-5;
        worstError = std::max(worstError, err);
        worstAllowed = std::max(worstAllowed, allowed);
        CHECK(err <= allowed);
      }
    }
  }
  std::printf("    worst error %.6f mm (budget %.6f mm)\n", worstError * 1000.0,
              worstAllowed * 1000.0);
}

void testScalarReadback() {
  beginTest("scalar field readback normalises over the requested range");
  auto cloud = std::make_shared<PointCloud>(loadFixture("helix_binary.pcd"));
  if (cloud->empty()) return;
  std::vector<Chunk> chunks = buildChunks(*cloud, smallChunks());
  const RamChunkSource source(cloud, chunks);

  int ringField = -1;
  for (int i = 0; i < static_cast<int>(cloud->fields.size()); ++i)
    if (cloud->fields[static_cast<std::size_t>(i)].name == "ring") ringField = i;
  CHECK(ringField >= 0);
  if (ringField < 0) return;

  const Chunk& chunk = chunks[0];
  std::vector<std::uint16_t> values(chunk.pointCount);
  CHECK(source.readChunkScalars(0, ringField, 0.0, 63.0, values));

  const ScalarColumn& column =
      cloud->columns[static_cast<std::size_t>(cloud->fields[static_cast<std::size_t>(ringField)].columnIndex)];
  for (std::uint32_t i = 0; i < chunk.pointCount; ++i) {
    const double expected = column.at(chunk.firstPoint + i) / 63.0 * 65535.0;
    CHECK_NEAR(values[i], expected, 2.0);
  }

  // A zero-width range must not divide by zero.
  CHECK(source.readChunkScalars(0, ringField, 5.0, 5.0, values));

  // x/y/z have no column but must still be readable as a height ramp, in
  // world coordinates so they match the statistics panel.
  int zField = -1;
  for (int i = 0; i < static_cast<int>(cloud->fields.size()); ++i)
    if (cloud->fields[static_cast<std::size_t>(i)].name == "z") zField = i;
  CHECK(zField >= 0);
  if (zField >= 0) CHECK(source.readChunkScalars(0, zField, -10.0, 10.0, values));
}

void testDegenerateClouds() {
  beginTest("degenerate inputs do not break chunking");
  // All points coincident: subdivision can never separate them, so the
  // depth limit has to stop the recursion.
  PointCloud cloud;
  cloud.positions.assign(10000, Vec3f{1.0f, 2.0f, 3.0f});
  cloud.sourceIndex.resize(10000);
  for (std::size_t i = 0; i < 10000; ++i) cloud.sourceIndex[i] = static_cast<std::uint32_t>(i);
  cloud.recomputeBounds();

  ChunkBuildStats stats;
  const std::vector<Chunk> chunks = buildChunks(cloud, smallChunks(1024), &stats);
  CHECK(!chunks.empty());
  std::uint64_t covered = 0;
  for (const Chunk& c : chunks) covered += c.pointCount;
  CHECK_EQ(covered, std::uint64_t{10000});
  for (const Chunk& c : chunks) CHECK(c.spacing >= 0.0f);

  // A single point.
  PointCloud one;
  one.positions.assign(1, Vec3f{0.0f, 0.0f, 0.0f});
  one.recomputeBounds();
  const std::vector<Chunk> oneChunk = buildChunks(one, smallChunks());
  CHECK_EQ(oneChunk.size(), std::size_t{1});
  CHECK_EQ(oneChunk[0].pointCount, std::uint32_t{1});

  // Empty.
  PointCloud none;
  CHECK(buildChunks(none, smallChunks()).empty());
}

}  // namespace

int main() {
  if (!std::filesystem::exists(fixtureDir())) {
    std::printf("fixtures missing; run: python3 tests/make_fixtures.py\n");
    return 2;
  }
  testCoverageAndContiguity();
  testPointsLieInsideTheirChunkBounds();
  testAttributesStayWithTheirPoints();
  testPrefixIsARepresentativeSample();
  testDeterminism();
  testQuantizationRoundTrip();
  testScalarReadback();
  testDegenerateClouds();

  std::printf("\n%d checks, \x1b[%sm%d failures\x1b[0m\n", g_checks, g_failures ? "31" : "32",
              g_failures);
  return g_failures == 0 ? 0 : 1;
}
