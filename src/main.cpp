#include <cstdio>
#include <cstdlib>
#include <string>

#include "app/viewer.hpp"
#include "core/log.hpp"

namespace pcv {
// Defined in the generated embedded_shaders.cpp; referenced here so the
// linker cannot discard that translation unit's static registrar.
void ensureEmbeddedShadersLinked();
}  // namespace pcv

namespace {

void usage() {
  std::printf(
      "pcd_viewer -- GPU point cloud viewer\n"
      "\n"
      "usage: pcd_viewer [options] [file ...]\n"
      "\n"
      "  --gpu auto|nvidia|intel   which GPU to render on (default: auto,\n"
      "                            which prefers the discrete one)\n"
      "  --size WxH                window size (default 1600x950)\n"
      "  --no-vsync                uncap the frame rate\n"
      "  --screenshot PATH         render one frame to a PNG and exit\n"
      "  --benchmark [N]           time N frames of a full orbit and exit\n"
      "  --nav orbit|fly|top       initial navigation mode\n"
      "  --yaw D --pitch D         initial camera angles, degrees\n"
      "  --zoom F                  move F times closer than the framed view\n"
      "  --density F               LOD target points per pixel (default 1.0)\n"
      "  --point-scale F           splat size multiplier (default 1.0)\n"
      "  --no-edl  --no-lod        disable eye-dome lighting / LOD\n"
      "  --chunk-size N            max points per chunk (default 131072)\n"
      "  --keep-invalid            keep points with NaN/Inf coordinates\n"
      "  --threads N               worker threads for loading\n"
      "  -v, --verbose             debug logging\n"
      "  -h, --help                this message\n"
      "\n"
      "controls:\n"
      "  left drag        orbit            shift+left / middle / right drag   pan\n"
      "  scroll           zoom             1 / 2 / 3   orbit / fly / top-down\n"
      "  W A S D Q E      fly              shift / alt  faster / slower\n"
      "  F  frame all     P  pick point    L  toggle LOD    E  toggle EDL\n"
      "  C  cycle colour  R  reload shaders               Esc  quit\n");
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
  pcv::ensureEmbeddedShadersLinked();

  pcv::ViewerOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") { usage(); return 0; }
    if (arg == "-v" || arg == "--verbose") { pcv::setLogLevel(pcv::LogLevel::Debug); continue; }
    if (arg == "--no-vsync") { options.vsync = false; continue; }
    if (arg == "--keep-invalid") { options.loadOptions.dropInvalid = false; continue; }
    if (arg == "--gpu" && i + 1 < argc) {
      if (!pcv::parseGpuPreference(argv[++i], options.gpu)) {
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
      if (v == "orbit") options.nav = pcv::NavMode::Orbit;
      else if (v == "fly") options.nav = pcv::NavMode::Fly;
      else if (v == "top") options.nav = pcv::NavMode::TopDown;
      else { std::fprintf(stderr, "unknown --nav '%s'\n", v.c_str()); return 2; }
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

  pcv::Viewer viewer;
  return viewer.run(options);
}
