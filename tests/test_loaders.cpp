// Loader tests. No framework dependency -- the whole harness is the few
// macros below, which keeps the build self-contained.
//
// The central claim being tested is that every decode path produces the same
// cloud. helix_compressed.pcd is written by PCL's own converter, so the
// binary_compressed test is an interoperability check against the reference
// implementation rather than against our own writer.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "io/cloud_builder.hpp"
#include "io/loader.hpp"
#include "io/laz_decode.hpp"
#include "io/lzf.hpp"

namespace {

int g_failures = 0;
int g_checks = 0;
std::string g_currentTest;

void fail(const char* file, int line, const std::string& what) {
  ++g_failures;
  std::printf("  \x1b[31mFAIL\x1b[0m %s:%d  %s\n", file, line, what.c_str());
}

#define CHECK(cond)                                                       \
  do {                                                                    \
    ++g_checks;                                                           \
    if (!(cond)) fail(__FILE__, __LINE__, "expected: " #cond);            \
  } while (0)

#define CHECK_EQ(a, b)                                                    \
  do {                                                                    \
    ++g_checks;                                                           \
    const auto va_ = (a);                                                 \
    const auto vb_ = (b);                                                 \
    if (!(va_ == vb_))                                                    \
      fail(__FILE__, __LINE__,                                            \
           std::string(#a " == " #b "  (") + std::to_string(va_) + " vs " \
           + std::to_string(vb_) + ")");                                  \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                             \
  do {                                                                    \
    ++g_checks;                                                           \
    const double va_ = static_cast<double>(a);                            \
    const double vb_ = static_cast<double>(b);                            \
    if (!(std::fabs(va_ - vb_) <= (eps)))                                 \
      fail(__FILE__, __LINE__,                                            \
           std::string(#a " ~= " #b "  (") + std::to_string(va_) + " vs " \
           + std::to_string(vb_) + ")");                                  \
  } while (0)

void beginTest(const char* name) {
  g_currentTest = name;
  std::printf("\x1b[1m%s\x1b[0m\n", name);
}

std::filesystem::path fixtureDir() {
  if (const char* env = std::getenv("PCV_FIXTURES")) return env;
  return std::filesystem::path(__FILE__).parent_path() / "fixtures";
}

using namespace pcv;

bool fixtureExists(const std::string& name) {
  return std::filesystem::exists(fixtureDir() / name);
}

PointCloud load(const std::string& name, LoadOptions options = {}) {
  PointCloud cloud;
  const std::string path = (fixtureDir() / name).string();
  const LoadResult r = loadPointCloud(path, options, cloud);
  if (!r.ok) {
    fail(__FILE__, __LINE__, "load failed for " + name + ": " + r.error);
    cloud.clear();
  }
  return cloud;
}

// Compares two clouds that should hold the same points. `posEps` of 0
// demands bit-identical positions.
void expectSameCloud(const PointCloud& a, const PointCloud& b, double posEps,
                     const char* label) {
  std::printf("  comparing %s\n", label);
  CHECK_EQ(a.size(), b.size());
  if (a.size() != b.size()) return;

  CHECK_EQ(a.columns.size(), b.columns.size());
  CHECK_EQ(a.fields.size(), b.fields.size());
  // Both must have chosen the same origin, or world coordinates will not
  // line up even when local ones do.
  CHECK_NEAR(a.origin.x, b.origin.x, 1e-9);
  CHECK_NEAR(a.origin.y, b.origin.y, 1e-9);
  CHECK_NEAR(a.origin.z, b.origin.z, 1e-9);

  std::size_t posMismatch = 0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const Vec3 pa = a.worldPosition(i);
    const Vec3 pb = b.worldPosition(i);
    if (std::fabs(pa.x - pb.x) > posEps || std::fabs(pa.y - pb.y) > posEps ||
        std::fabs(pa.z - pb.z) > posEps) {
      if (posMismatch < 3)
        std::printf("    point %zu: (%.9f %.9f %.9f) vs (%.9f %.9f %.9f)\n", i, pa.x, pa.y, pa.z,
                    pb.x, pb.y, pb.z);
      ++posMismatch;
    }
  }
  CHECK_EQ(posMismatch, std::size_t{0});

  const std::size_t nCols = std::min(a.columns.size(), b.columns.size());
  for (std::size_t c = 0; c < nCols; ++c) {
    CHECK_EQ(static_cast<int>(a.columns[c].type()), static_cast<int>(b.columns[c].type()));
    std::size_t mismatch = 0;
    for (std::size_t i = 0; i < a.size(); ++i)
      if (a.columns[c].at(i) != b.columns[c].at(i)) ++mismatch;
    if (mismatch != 0)
      std::printf("    column %zu (%s) differs at %zu points\n", c,
                  c < a.fields.size() ? a.fields[c].name.c_str() : "?", mismatch);
    CHECK_EQ(mismatch, std::size_t{0});
  }
}

void testEncodingsAgree() {
  beginTest("every PCD encoding decodes to the same cloud");
  const PointCloud ascii = load("helix_ascii.pcd");
  const PointCloud binary = load("helix_binary.pcd");
  if (binary.empty()) return;

  CHECK_EQ(binary.size(), std::size_t{5000});

  // ASCII went through a decimal round-trip, so allow a float32 ulp.
  expectSameCloud(binary, ascii, 1e-5, "binary vs ascii");

  // The binary_compressed fixture is written by PCL, so it is only present
  // where PCL is installed. Skipping rather than failing keeps the suite
  // usable on a machine -- or a CI runner -- without it.
  if (!fixtureExists("helix_compressed.pcd")) {
    std::printf("  skipping binary_compressed (fixture needs PCL to generate)\n");
    return;
  }
  const PointCloud compressed = load("helix_compressed.pcd");
  if (compressed.empty()) return;
  // Both hold the identical float32 bits, so this must match exactly.
  expectSameCloud(binary, compressed, 0.0, "binary vs binary_compressed (PCL-written)");
}

void testFieldTypesPreserved() {
  beginTest("declared field types survive the load");
  const PointCloud c = load("helix_binary.pcd");
  if (c.empty()) return;

  struct Expect { const char* name; FieldType type; FieldRole role; };
  const Expect expected[] = {
      {"x", FieldType::F32, FieldRole::Position},
      {"y", FieldType::F32, FieldRole::Position},
      {"z", FieldType::F32, FieldRole::Position},
      {"intensity", FieldType::U16, FieldRole::Intensity},
      {"ring", FieldType::U8, FieldRole::Ring},
      {"label", FieldType::U32, FieldRole::Label},
      {"t", FieldType::F64, FieldRole::Time},
  };
  CHECK_EQ(c.fields.size(), std::size(expected));
  for (const Expect& e : expected) {
    const FieldInfo* f = c.findField(e.name);
    CHECK(f != nullptr);
    if (f == nullptr) continue;
    CHECK_EQ(static_cast<int>(f->type), static_cast<int>(e.type));
    CHECK_EQ(static_cast<int>(f->role), static_cast<int>(e.role));
  }

  // The float64 timestamp is the case that a float32 store would destroy:
  // at Unix-epoch magnitudes float32 resolves to ~128 seconds, so the 0.1 ms
  // step between consecutive points would vanish entirely.
  //
  // Even float64 is not exact here. At 1.7e9 its ulp is 2^31 * 2^-52 =
  // 4.77e-7 s, so a 0.1 ms step round-trips with ~1e-7 of error. That is the
  // floor for absolute epoch timestamps in any 64-bit float, and the
  // tolerance below reflects it rather than papering over it.
  constexpr double kEpochUlp = 1e-6;
  const FieldInfo* t = c.findField("t");
  CHECK(t != nullptr && t->columnIndex >= 0);
  if (t != nullptr && t->columnIndex >= 0) {
    const ScalarColumn& col = c.columns[static_cast<std::size_t>(t->columnIndex)];
    CHECK_NEAR(col.at(0), 1700000000.0, kEpochUlp);
    CHECK_NEAR(col.at(1) - col.at(0), 1e-4, kEpochUlp);
    CHECK_NEAR(col.at(4999), 1700000000.0 + 4999e-4, kEpochUlp);
    // The point of the exercise: 5000 steps of 0.1 ms must still be
    // distinguishable from the start, which float32 could not manage.
    CHECK(col.at(4999) - col.at(0) > 0.49);
  }

  // Narrow integer columns stay narrow rather than widening to float.
  const FieldInfo* ring = c.findField("ring");
  if (ring != nullptr && ring->columnIndex >= 0)
    CHECK_EQ(c.columns[static_cast<std::size_t>(ring->columnIndex)].elementSize(), std::size_t{1});
}

void testInvalidPointsDropped() {
  beginTest("non-finite points are dropped and counted");
  const PointCloud c = load("helix_nan_binary.pcd");
  if (c.empty()) return;

  // The generator poisons every 97th point of 5000: indices 0, 97, ... 4850.
  CHECK_EQ(c.invalidPointsDropped, std::uint64_t{52});
  CHECK_EQ(c.size(), std::size_t{5000 - 52});

  for (std::size_t i = 0; i < c.size(); ++i) CHECK(positionFinite(c.positions[i]));
  CHECK(c.boundsWorld.valid());
  // A single surviving NaN would have poisoned the box into non-finite.
  CHECK(std::isfinite(c.boundsWorld.lo.x) && std::isfinite(c.boundsWorld.hi.x));

  // Compaction must keep every parallel array in step: the survivors' source
  // rows are exactly the non-multiples of 97.
  CHECK(c.hasSourceIndex());
  if (c.hasSourceIndex()) {
    CHECK_EQ(c.sourceIndex[0], std::uint32_t{1});
    CHECK_EQ(c.sourceIndex[95], std::uint32_t{96});
    CHECK_EQ(c.sourceIndex[96], std::uint32_t{98});  // 97 was dropped
  }
}

void testKeepInvalid() {
  beginTest("--keep-invalid retains every point");
  LoadOptions options;
  options.dropInvalid = false;
  const PointCloud c = load("helix_nan_binary.pcd", options);
  if (c.empty()) return;
  CHECK_EQ(c.size(), std::size_t{5000});
  CHECK_EQ(c.invalidPointsDropped, std::uint64_t{0});
}

void testPackedRgb() {
  beginTest("packed RGB inside a float field unpacks correctly");
  const PointCloud c = load("helix_rgb.pcd");
  if (c.empty()) return;

  CHECK(c.hasColors());
  CHECK_EQ(c.size(), std::size_t{5000});
  for (std::size_t i : {std::size_t{0}, std::size_t{1}, std::size_t{37}, std::size_t{4999}}) {
    const std::uint8_t r = static_cast<std::uint8_t>((i * 7) % 256);
    const std::uint8_t g = static_cast<std::uint8_t>((i * 13) % 256);
    const std::uint8_t b = static_cast<std::uint8_t>((i * 29) % 256);
    const std::uint32_t got = c.colors[i];
    CHECK_EQ(got & 0xFFu, static_cast<std::uint32_t>(r));
    CHECK_EQ((got >> 8) & 0xFFu, static_cast<std::uint32_t>(g));
    CHECK_EQ((got >> 16) & 0xFFu, static_cast<std::uint32_t>(b));
    CHECK_EQ((got >> 24) & 0xFFu, std::uint32_t{255});
  }
  CHECK(c.colorStatsValid);
}

void testKittiBin() {
  beginTest("raw KITTI .bin");
  const PointCloud c = load("helix.bin");
  if (c.empty()) return;
  CHECK_EQ(c.size(), std::size_t{5000});
  CHECK_EQ(c.fields.size(), std::size_t{4});
  const FieldInfo* f = c.findField("intensity");
  CHECK(f != nullptr);
  if (f != nullptr) CHECK_EQ(static_cast<int>(f->role), static_cast<int>(FieldRole::Intensity));
}

void testFormatDetection() {
  beginTest("format detection");
  CHECK_EQ(static_cast<int>(detectFormat((fixtureDir() / "helix_binary.pcd").string())),
           static_cast<int>(CloudFormat::Pcd));
  if (fixtureExists("helix_compressed.pcd"))
    CHECK_EQ(static_cast<int>(detectFormat((fixtureDir() / "helix_compressed.pcd").string())),
             static_cast<int>(CloudFormat::Pcd));
  CHECK_EQ(static_cast<int>(detectFormat((fixtureDir() / "helix.bin").string())),
           static_cast<int>(CloudFormat::KittiBin));
  CHECK_EQ(static_cast<int>(detectFormat((fixtureDir() / "helix_le.ply").string())),
           static_cast<int>(CloudFormat::Ply));
  CHECK_EQ(static_cast<int>(detectFormat((fixtureDir() / "helix_ascii.ply").string())),
           static_cast<int>(CloudFormat::Ply));
  CHECK_EQ(static_cast<int>(detectFormat((fixtureDir() / "helix_12_fmt3.las").string())),
           static_cast<int>(CloudFormat::Las));
}

void testStatsAgainstKnownValues() {
  beginTest("statistics match hand-computed values");
  const PointCloud c = load("helix_binary.pcd");
  if (c.empty()) return;

  // ring cycles 0..63 uniformly over 5000 points.
  const FieldInfo* ring = c.findField("ring");
  CHECK(ring != nullptr);
  if (ring != nullptr) {
    CHECK_EQ(ring->stats.min, 0.0);
    CHECK_EQ(ring->stats.max, 63.0);
    CHECK(ring->stats.discrete);
    CHECK_EQ(ring->stats.distinctValues.size(), std::size_t{64});
  }

  // z ramps linearly as i/n * 20 - 10 for i in [0, 5000), so it reaches
  // -10 exactly but stops one step short of +10 at 9.996. The mean sits at
  // the midpoint and the spread is that of a uniform distribution,
  // range / sqrt(12).
  constexpr double kZMin = -10.0;
  constexpr double kZMax = 4999.0 / 5000.0 * 20.0 - 10.0;  // 9.996
  const FieldInfo* z = c.findField("z");
  CHECK(z != nullptr);
  if (z != nullptr) {
    CHECK_NEAR(z->stats.min, kZMin, 1e-3);
    CHECK_NEAR(z->stats.max, kZMax, 1e-3);
    CHECK_NEAR(z->stats.mean, (kZMin + kZMax) * 0.5, 1e-2);
    CHECK_NEAR(z->stats.stddev, (kZMax - kZMin) / std::sqrt(12.0), 1e-2);
  }

  std::uint64_t histTotal = 0;
  if (z != nullptr) for (std::uint32_t v : z->stats.histogram) histTotal += v;
  CHECK_EQ(histTotal, std::uint64_t{5000});
}

void testLzfEdgeCases() {
  beginTest("LZF decoder rejects malformed streams");
  std::uint8_t out[64];
  // A literal run claiming more bytes than the input holds.
  const std::uint8_t truncated[] = {0x1F};
  CHECK_EQ(lzfDecompress(truncated, sizeof(truncated), out, sizeof(out)), std::size_t{0});
  // A back-reference pointing before the start of the output.
  const std::uint8_t badRef[] = {0xE0, 0x00, 0xFF};
  CHECK_EQ(lzfDecompress(badRef, sizeof(badRef), out, sizeof(out)), std::size_t{0});
  // A valid single-byte literal.
  const std::uint8_t ok[] = {0x00, 0x42};
  CHECK_EQ(lzfDecompress(ok, sizeof(ok), out, sizeof(out)), std::size_t{1});
  CHECK_EQ(static_cast<int>(out[0]), 0x42);
}

// Every PLY encoding must agree, and must agree with the PCD holding the
// same points. The big-endian case in particular exercises byte swapping on
// every field width.
void testPlyEncodingsAgree() {
  beginTest("every PLY encoding decodes to the same cloud");
  const PointCloud ascii = load("helix_ascii.ply");
  const PointCloud little = load("helix_le.ply");
  const PointCloud big = load("helix_be.ply");
  const PointCloud faces = load("helix_faces.ply");
  const PointCloud leading = load("helix_leading.ply");
  if (little.empty()) return;

  CHECK_EQ(little.size(), std::size_t{5000});
  expectSameCloud(little, big, 0.0, "little-endian vs big-endian");
  expectSameCloud(little, ascii, 1e-5, "little-endian vs ascii");
  // A trailing face element must not disturb the vertex block.
  expectSameCloud(little, faces, 0.0, "with a face element appended");
  // An element declared BEFORE vertex means the vertex block does not start
  // at the payload offset; the reader has to skip past it.
  expectSameCloud(little, leading, 0.0, "with an element preceding the vertices");

  CHECK(little.hasColors());
  const FieldInfo* intensity = little.findField("intensity");
  CHECK(intensity != nullptr);
  if (intensity != nullptr)
    CHECK_EQ(static_cast<int>(intensity->type), static_cast<int>(FieldType::U16));
}

void testPlyMatchesPcd() {
  beginTest("PLY and PCD of the same points agree");
  const PointCloud ply = load("helix_le.ply");
  const PointCloud pcd = load("helix_binary.pcd");
  if (ply.empty() || pcd.empty()) return;

  CHECK_EQ(ply.size(), pcd.size());
  std::size_t mismatch = 0;
  for (std::size_t i = 0; i < std::min(ply.size(), pcd.size()); ++i) {
    const Vec3 a = ply.worldPosition(i);
    const Vec3 b = pcd.worldPosition(i);
    if (std::fabs(a.x - b.x) > 1e-6 || std::fabs(a.y - b.y) > 1e-6 ||
        std::fabs(a.z - b.z) > 1e-6)
      ++mismatch;
  }
  CHECK_EQ(mismatch, std::size_t{0});
}

void testLas() {
  beginTest("LAS point formats, scaled integer coordinates and classification");
  const PointCloud f3 = load("helix_12_fmt3.las");   // LAS 1.2, GPS time + RGB
  const PointCloud f7 = load("helix_14_fmt7.las");   // LAS 1.4, 64-bit count
  const PointCloud f0 = load("helix_12_fmt0.las");   // minimal format
  if (f3.empty()) return;

  CHECK_EQ(f3.size(), std::size_t{5000});
  CHECK_EQ(f7.size(), std::size_t{5000});
  CHECK_EQ(f0.size(), std::size_t{5000});

  // Coordinates are int32 scaled by 0.001, so they round-trip to a
  // millimetre and no better.
  const PointCloud pcd = load("helix_binary.pcd");
  if (!pcd.empty()) {
    std::size_t mismatch = 0;
    for (std::size_t i = 0; i < f3.size(); ++i) {
      const Vec3 a = f3.worldPosition(i);
      const Vec3 b = pcd.worldPosition(i);
      if (std::fabs(a.x - b.x) > 1e-3 || std::fabs(a.y - b.y) > 1e-3 ||
          std::fabs(a.z - b.z) > 1e-3)
        ++mismatch;
    }
    CHECK_EQ(mismatch, std::size_t{0});
  }

  // Formats 0-5 pack synthetic/key-point/withheld flags into the top three
  // bits of the classification byte; the fixture sets one every 50 points.
  // Unmasked, a class of 6 would read as 134.
  const FieldInfo* classification = f3.findField("classification");
  CHECK(classification != nullptr);
  if (classification != nullptr) {
    CHECK(classification->stats.max <= 31.0);
    CHECK_EQ(static_cast<int>(classification->role), static_cast<int>(FieldRole::Label));
  }

  // GPS time is float64 and must survive as such.
  const FieldInfo* gps = f3.findField("gps_time");
  CHECK(gps != nullptr);
  if (gps != nullptr) {
    CHECK_EQ(static_cast<int>(gps->type), static_cast<int>(FieldType::F64));
    CHECK_NEAR(gps->stats.min, 1700000000.0, 1e-5);
  }

  // Format 0 carries no colour or GPS time.
  CHECK(!f0.hasColors());
  CHECK(f0.findField("gps_time") == nullptr);
  CHECK(f3.hasColors());
  CHECK(f7.hasColors());
}

void testLasColorDepthHeuristic() {
  beginTest("LAS 16-bit and 8-bit colour channels both land on the same colours");
  // LAS declares colour as 16-bit, but many files written from 8-bit
  // imagery leave the values in 0..255. Guessing wrong makes a cloud either
  // black or saturated, so the reader samples to decide.
  const PointCloud wide = load("helix_12_fmt3.las");      // values scaled by 257
  const PointCloud narrow = load("helix_8bitcolor.las");  // values left at 0..255
  const PointCloud reference = load("helix_rgb.pcd");
  if (wide.empty() || narrow.empty() || reference.empty()) return;

  std::size_t mismatchWide = 0;
  std::size_t mismatchNarrow = 0;
  for (std::size_t i = 0; i < reference.size(); ++i) {
    if (wide.colors[i] != reference.colors[i]) ++mismatchWide;
    if (narrow.colors[i] != reference.colors[i]) ++mismatchNarrow;
  }
  CHECK_EQ(mismatchWide, std::size_t{0});
  CHECK_EQ(mismatchNarrow, std::size_t{0});
}

// LAZ must produce exactly the cloud its LAS original does. The reader
// serialises each decompressed point back into the on-disk record layout
// so both flow through the same decode plan; this is the test that keeps
// those two paths from drifting.
void testLazMatchesLas() {
  beginTest("LAZ decodes identically to the LAS it was compressed from");
  if (!lazSupported()) {
    std::printf("  skipping (this build has no laszip)\n");
    return;
  }
  const char* pairs[][2] = {
      {"helix_12_fmt3.las", "helix_12_fmt3.laz"},
      {"helix_14_fmt7.las", "helix_14_fmt7.laz"},
      {"helix_12_fmt0.las", "helix_12_fmt0.laz"},
  };

  for (const auto& pair : pairs) {
    if (!std::filesystem::exists(fixtureDir() / pair[1])) {
      std::printf("  skipping %s (no laszip at build time)\n", pair[1]);
      continue;
    }
    const PointCloud las = load(pair[0]);
    const PointCloud laz = load(pair[1]);
    if (las.empty() || laz.empty()) continue;

    // Positions came from identical int32s through identical scale and
    // offset, so this must match bit for bit.
    expectSameCloud(las, laz, 0.0, pair[1]);

    CHECK_EQ(las.hasColors(), laz.hasColors());
    if (las.hasColors() && laz.hasColors()) {
      std::size_t mismatch = 0;
      for (std::size_t i = 0; i < las.size(); ++i)
        if (las.colors[i] != laz.colors[i]) ++mismatch;
      CHECK_EQ(mismatch, std::size_t{0});
    }
  }
}

void testLazFormatName() {
  beginTest("LAZ reports itself as compressed");
  if (!std::filesystem::exists(fixtureDir() / "helix_12_fmt3.laz")) return;
  if (!lazSupported()) {
    std::printf("  skipping (this build has no laszip)\n");
    return;
  }
  const PointCloud laz = load("helix_12_fmt3.laz");
  if (laz.empty()) return;
  CHECK(laz.formatName.find("LAZ") != std::string::npos);
  // Detection is by the compression bits in the header's format byte, not
  // by the extension, so a .laz named .las still reads correctly.
  CHECK_EQ(static_cast<int>(detectFormat((fixtureDir() / "helix_12_fmt3.laz").string())),
           static_cast<int>(CloudFormat::Las));
}

// A build without laszip must refuse a LAZ file cleanly and say what to do
// about it, rather than crashing or returning an empty cloud that looks
// like a corrupt file.
void testLazWithoutLaszipIsRefusedGracefully() {
  beginTest("a build without laszip refuses LAZ with instructions");
  if (lazSupported()) {
    std::printf("  skipping (this build has laszip)\n");
    return;
  }
  if (!std::filesystem::exists(fixtureDir() / "helix_12_fmt3.laz")) return;

  PointCloud cloud;
  const LoadResult r =
      loadPointCloud((fixtureDir() / "helix_12_fmt3.laz").string(), {}, cloud);
  CHECK(!r.ok);
  CHECK(!r.cancelled);
  CHECK(cloud.empty());
  // The message has to name the fix, not just the failure.
  CHECK(r.error.find("laszip") != std::string::npos);

  // And the uncompressed sibling must still read, so a missing optional
  // dependency costs only the one format.
  const PointCloud las = load("helix_12_fmt3.las");
  CHECK_EQ(las.size(), std::size_t{5000});
}

void testGreyscaleDetection() {
  beginTest("greyscale colour is detected per point, not from channel statistics");
  const PointCloud grey = load("helix_greyscale.pcd");
  const PointCloud colour = load("helix_rgb.pcd");
  if (grey.empty() || colour.empty()) return;

  CHECK(grey.colorIsGreyscale);
  CHECK_NEAR(grey.colorGreyscaleFraction, 1.0, 1e-9);

  // The colour fixture cycles R, G and B at different rates, so the three
  // channels have near-identical means and standard deviations while being
  // different on every single point. Comparing aggregate statistics -- the
  // obvious test -- calls this greyscale, which is why it is done per point.
  CHECK(!colour.colorIsGreyscale);
  const FieldStats& r = colour.colorChannelStats[0];
  const FieldStats& g = colour.colorChannelStats[1];
  CHECK(std::fabs(r.mean - g.mean) < 1.0);  // the trap the old test fell into
}

void testMissingFileIsAnError() {
  beginTest("missing and malformed inputs are reported, not crashed on");
  PointCloud cloud;
  LoadResult r = loadPointCloud("/definitely/not/here.pcd", {}, cloud);
  CHECK(!r.ok);
  CHECK(!r.error.empty());
}

}  // namespace

int main() {
  if (!std::filesystem::exists(fixtureDir())) {
    std::printf("fixtures missing at %s\nrun: python3 tests/make_fixtures.py\n",
                fixtureDir().c_str());
    return 2;
  }

  testEncodingsAgree();
  testFieldTypesPreserved();
  testInvalidPointsDropped();
  testKeepInvalid();
  testPackedRgb();
  testKittiBin();
  testFormatDetection();
  testPlyEncodingsAgree();
  testPlyMatchesPcd();
  testLas();
  testLasColorDepthHeuristic();
  testLazMatchesLas();
  testLazFormatName();
  testLazWithoutLaszipIsRefusedGracefully();
  testGreyscaleDetection();
  testStatsAgainstKnownValues();
  testLzfEdgeCases();
  testMissingFileIsAnError();

  std::printf("\n%d checks, \x1b[%sm%d failures\x1b[0m\n", g_checks, g_failures ? "31" : "32",
              g_failures);
  return g_failures == 0 ? 0 : 1;
}
