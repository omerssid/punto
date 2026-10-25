#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "app/viewer.hpp"
#include "core/log.hpp"
#include "core/memory.hpp"

namespace punto {
// Defined in the generated embedded_shaders.cpp; referenced here so the
// linker cannot discard that translation unit's static registrar.
void ensureEmbeddedShadersLinked();
}  // namespace punto

namespace {

void usage() {
  std::printf(
      "punto-viewer -- GPU point cloud viewer\n"
      "\n"
      "usage: punto-viewer [options] [file ...]\n"
      "\n"
      "  --gpu auto|nvidia|intel   which GPU to render on (default: auto,\n"
      "                            which prefers the discrete one)\n"
      "  --size WxH                window size (default 1600x950)\n"
      "  --no-vsync                uncap the frame rate\n"
      "  --screenshot PATH         render one frame to a PNG and exit\n"
      "  --benchmark [N]           time N frames of a full orbit and exit\n"
      "  --resize-stress [N]       sweep the viewport size and report reallocations\n"
      "  --nav orbit|fly|top       initial navigation mode\n"
      "  --view NAME               top|bottom|+x|-x|+y|-y|iso\n"
      "  --yaw D --pitch D         initial camera angles, degrees\n"
      "  --zoom F                  move F times closer than the framed view\n"
      "  --density F               LOD target points per pixel (default 1.0)\n"
      "  --point-scale F           splat size multiplier (default 1.0)\n"
      "  --no-edl  --no-lod        disable eye-dome lighting / LOD\n"
      "  --color rgb|scalar|flat|chunk   colour source\n"
      "  --field NAME              scalar field to colour by\n"
      "  --ramp NAME               viridis|turbo|grayscale|coolwarm|plasma|spectral\n"
      "  --pick X,Y                pick at this screen position before capturing\n"
      "  --measure X,Y             add a measurement point (repeatable)\n"
      "  --crop F                  crop to fraction F of the cloud's bounds\n"
      "  --chunk-size N            max points per chunk (default 131072)\n"
      "  --keep-invalid            keep points with NaN/Inf coordinates\n"
      "  --force                   open even if the pre-flight estimate says\n"
      "                            the file will not fit in available memory\n"
      "  --max-points N            keep at most N points, evenly spread through\n"
      "                            the file, to open a cloud that will not fit\n"
      "  --drop-fields A,B         do not decode these scalar fields at all\n"
      "  --threads N               worker threads for loading\n"
      "  -v, --verbose             debug logging\n"
      "  -h, --help                this message\n"
      "\n"
      "controls:\n"
      "  left drag        orbit            middle / right drag   pan\n"
      "  shift + click    select a point   double click  select and focus\n"
      "  Tab              show/hide panels\n"
      "  scroll           zoom             1 / 2 / 3   orbit / fly / top-down\n"
      "  W A S D Q E      fly              shift / alt  faster / slower\n"
      "  F  frame all     L  toggle LOD     E  toggle EDL\n"
      "  C  cycle colour  R  reload shaders               Esc  quit\n");
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

bool parseSize(const std::string& text, int& width, int& height) {
  const std::size_t x = text.find('x');
  if (x == std::string::npos) return false;
  width = std::atoi(text.substr(0, x).c_str());
  height = std::atoi(text.substr(x + 1).c_str());
  return width > 0 && height > 0;
}

}  // namespace

int main(int argc, char** argv) {
  // Make this process the OOM killer's first choice. A viewer that asked for
  // too much is far more expendable than the session around it; left at the
  // default it scores below a browser, so an oversized open gets the browser
  // killed and leaves the viewer thrashing.
  punto::preferSelfForOomKill();

  punto::ensureEmbeddedShadersLinked();

  punto::ViewerOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") { usage(); return 0; }
    if (arg == "-v" || arg == "--verbose") { punto::setLogLevel(punto::LogLevel::Debug); continue; }
    if (arg == "--no-vsync") { options.vsync = false; continue; }
    if (arg == "--keep-invalid") { options.loadOptions.dropInvalid = false; continue; }
    if (arg == "--force") { options.forceLoad = true; continue; }
    if (arg == "--max-points" && i + 1 < argc) {
      options.loadOptions.maxPoints = std::strtoull(argv[++i], nullptr, 10);
      continue;
    }
    if (arg == "--drop-fields" && i + 1 < argc) {
      options.loadOptions.dropFields = splitFieldList(argv[++i]);
      continue;
    }
    if (arg == "--gpu" && i + 1 < argc) {
      if (!punto::parseGpuPreference(argv[++i], options.gpu)) {
        std::fprintf(stderr, "unknown --gpu value '%s'\n", argv[i]);
        return 2;
      }
      continue;
    }
    if (arg == "--size" && i + 1 < argc) {
      if (!parseSize(argv[++i], options.width, options.height)) {
        std::fprintf(stderr, "bad --size '%s', expected WxH\n", argv[i]);
        return 2;
      }
      continue;
    }
    if (arg == "--screenshot" && i + 1 < argc) { options.screenshotPath = argv[++i]; continue; }
    if (arg == "--nav" && i + 1 < argc) {
      const std::string v = argv[++i];
      if (v == "orbit") options.nav = punto::NavMode::Orbit;
      else if (v == "fly") options.nav = punto::NavMode::Fly;
      else if (v == "top") options.nav = punto::NavMode::TopDown;
      else { std::fprintf(stderr, "unknown --nav '%s'\n", v.c_str()); return 2; }
      continue;
    }
    if (arg == "--view" && i + 1 < argc) {
      const std::string v = argv[++i];
      const struct { const char* name; punto::StandardView view; } kViews[] = {
          {"top", punto::StandardView::Top},       {"bottom", punto::StandardView::Bottom},
          {"+x", punto::StandardView::PlusX},      {"-x", punto::StandardView::MinusX},
          {"+y", punto::StandardView::PlusY},      {"-y", punto::StandardView::MinusY},
          {"iso", punto::StandardView::Isometric}};
      bool found = false;
      for (const auto& candidate : kViews)
        if (v == candidate.name) { options.standardView = candidate.view; found = true; }
      if (!found) {
        std::fprintf(stderr, "unknown --view '%s' (top|bottom|+x|-x|+y|-y|iso)\n", v.c_str());
        return 2;
      }
      options.haveStandardView = true;
      continue;
    }
    if (arg == "--yaw" && i + 1 < argc) {
      options.yawDegrees = static_cast<float>(std::atof(argv[++i]));
      options.haveYaw = true;
      continue;
    }
    if (arg == "--pitch" && i + 1 < argc) {
      options.pitchDegrees = static_cast<float>(std::atof(argv[++i]));
      options.havePitch = true;
      continue;
    }
    if (arg == "--zoom" && i + 1 < argc) {
      options.zoom = static_cast<float>(std::atof(argv[++i]));
      continue;
    }
    if (arg == "--color" && i + 1 < argc) {
      const std::string v = argv[++i];
      if (v == "rgb") options.render.colorMode = punto::ColorMode::Rgb;
      else if (v == "scalar") options.render.colorMode = punto::ColorMode::Scalar;
      else if (v == "flat") options.render.colorMode = punto::ColorMode::Flat;
      else if (v == "chunk") options.render.colorMode = punto::ColorMode::ChunkId;
      else { std::fprintf(stderr, "unknown --color '%s'\n", v.c_str()); return 2; }
      options.colorModeExplicit = true;
      continue;
    }
    if (arg == "--field" && i + 1 < argc) { options.fieldName = argv[++i]; continue; }
    if (arg == "--measure" && i + 1 < argc) {
      const std::string v = argv[++i];
      const std::size_t comma = v.find(',');
      if (comma == std::string::npos) {
        std::fprintf(stderr, "--measure expects X,Y\n");
        return 2;
      }
      options.measureAt.emplace_back(std::atoi(v.substr(0, comma).c_str()),
                                     std::atoi(v.substr(comma + 1).c_str()));
      continue;
    }
    if (arg == "--crop" && i + 1 < argc) {
      options.cropFraction = static_cast<float>(std::atof(argv[++i]));
      continue;
    }
    if (arg == "--pick" && i + 1 < argc) {
      const std::string v = argv[++i];
      const std::size_t comma = v.find(',');
      if (comma == std::string::npos) {
        std::fprintf(stderr, "--pick expects X,Y\n");
        return 2;
      }
      options.pickX = std::atoi(v.substr(0, comma).c_str());
      options.pickY = std::atoi(v.substr(comma + 1).c_str());
      continue;
    }
    if (arg == "--ramp" && i + 1 < argc) {
      const std::string v = argv[++i];
      const char* names[] = {"viridis", "turbo", "grayscale", "coolwarm", "plasma", "spectral"};
      bool found = false;
      for (int r = 0; r < 6; ++r)
        if (v == names[r]) { options.render.ramp = static_cast<punto::ColorRamp>(r); found = true; }
      if (!found) { std::fprintf(stderr, "unknown --ramp '%s'\n", v.c_str()); return 2; }
      continue;
    }
    if (arg == "--no-edl") { options.render.edlEnabled = false; continue; }
    if (arg == "--no-lod") { options.render.lodEnabled = false; continue; }
    if (arg == "--density" && i + 1 < argc) {
      options.render.lodDensity = static_cast<float>(std::atof(argv[++i]));
      continue;
    }
    if (arg == "--point-scale" && i + 1 < argc) {
      options.render.pointScale = static_cast<float>(std::atof(argv[++i]));
      continue;
    }
    if (arg == "--resize-stress") {
      options.resizeStress = true;
      if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9')
        options.resizeStressFrames = std::atoi(argv[++i]);
      continue;
    }
    if (arg == "--benchmark") {
      options.benchmark = true;
      // Optional count, if the next argument is a bare number.
      if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9')
        options.benchmarkFrames = std::atoi(argv[++i]);
      continue;
    }
    if (arg == "--chunk-size" && i + 1 < argc) {
      options.chunkOptions.maxPointsPerChunk = static_cast<unsigned>(std::atoi(argv[++i]));
      continue;
    }
    if (arg == "--threads" && i + 1 < argc) {
      options.loadOptions.workers = static_cast<unsigned>(std::atoi(argv[++i]));
      options.chunkOptions.workers = options.loadOptions.workers;
      continue;
    }
    if (!arg.empty() && arg[0] == '-') {
      std::fprintf(stderr, "unknown option '%s'\n", arg.c_str());
      usage();
      return 2;
    }
    options.files.push_back(arg);
  }

  punto::Viewer viewer;
  return viewer.run(options);
}
