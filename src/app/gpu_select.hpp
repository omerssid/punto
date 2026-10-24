// Discrete-GPU selection, before any GL call.
#pragma once

#include <string>

namespace punto {

enum class GpuPreference { Auto, Discrete, Integrated };

bool parseGpuPreference(const std::string& text, GpuPreference& out);

// What the X server's RandR providers say about how displays are driven.
//
// On a hybrid laptop there are two interesting arrangements, and they want
// opposite things:
//
//  - The monitor hangs off the integrated GPU. Rendering on the discrete
//    one and copying the result across is a clear win.
//
//  - The monitor is wired to the discrete GPU but the X screen renders on
//    the integrated one, so frames are already copied the other way for
//    scanout ("reverse PRIME", a provider advertising Sink Output).
//    Forcing offload then makes every frame travel discrete -> integrated
//    -> discrete across PCIe, twice per frame. At 2560x1440 that is enough
//    to stall the compositor badly.
struct PrimeTopology {
  bool queried = false;
  bool multipleProviders = false;
  // A sink-output provider is actually driving a connected, lit monitor.
  bool reversePrimeActive = false;
  std::string sinkProviderName;
  std::string sourceProviderName;
};

// Inspects RandR providers. Safe to call before glfwInit; opens and closes
// its own X connection, and returns `queried = false` where RandR is
// unavailable (Wayland, headless, no Xrandr at build time).
PrimeTopology detectPrimeTopology();

// Must run before glfwInit().
//
// libglvnd picks its vendor when the GL stack is first touched and reads
// these variables at that moment, so setting them in-process is enough and
// no wrapper script is needed.
//
// Auto declines to force offload when reverse PRIME is driving a display,
// because there it makes things dramatically worse rather than better.
// An explicit --gpu nvidia always forces it.
void applyGpuPreference(GpuPreference preference, const PrimeTopology& topology);

}  // namespace punto
