#include "app/gpu_select.hpp"

#include <cstdlib>

#include "core/log.hpp"

#ifdef PCV_HAVE_XRANDR
#include <X11/Xlib.h>
#include <X11/extensions/Xrandr.h>
#endif

namespace pcv {

bool parseGpuPreference(const std::string& text, GpuPreference& out) {
  if (text == "auto") { out = GpuPreference::Auto; return true; }
  if (text == "nvidia" || text == "discrete" || text == "dgpu") {
    out = GpuPreference::Discrete;
    return true;
  }
  if (text == "intel" || text == "integrated" || text == "igpu") {
    out = GpuPreference::Integrated;
    return true;
  }
  return false;
}

#ifndef PCV_HAVE_XRANDR

PrimeTopology detectPrimeTopology() { return PrimeTopology{}; }

#else

PrimeTopology detectPrimeTopology() {
  PrimeTopology topology;

  Display* display = XOpenDisplay(nullptr);
  if (display == nullptr) return topology;

  const Window root = DefaultRootWindow(display);
  XRRProviderResources* providers = XRRGetProviderResources(display, root);
  XRRScreenResources* screen = XRRGetScreenResourcesCurrent(display, root);

  if (providers != nullptr && screen != nullptr) {
    topology.queried = true;
    topology.multipleProviders = providers->nproviders > 1;

    for (int i = 0; i < providers->nproviders; ++i) {
      XRRProviderInfo* info = XRRGetProviderInfo(display, screen, providers->providers[i]);
      if (info == nullptr) continue;

      const std::string name = info->name != nullptr ? info->name : "";
      if ((info->capabilities & RR_Capability_SourceOutput) != 0 &&
          topology.sourceProviderName.empty())
        topology.sourceProviderName = name;

      if ((info->capabilities & RR_Capability_SinkOutput) != 0) {
        // Only interesting if it is actually lighting up a monitor; a
        // capable but unused provider costs nothing.
        for (int o = 0; o < info->noutputs; ++o) {
          XRROutputInfo* output = XRRGetOutputInfo(display, screen, info->outputs[o]);
          if (output == nullptr) continue;
          const bool live = output->connection == RR_Connected && output->crtc != None;
          XRRFreeOutputInfo(output);
          if (live) {
            topology.reversePrimeActive = true;
            topology.sinkProviderName = name;
            break;
          }
        }
      }
      XRRFreeProviderInfo(info);
    }
  }

  if (providers != nullptr) XRRFreeProviderResources(providers);
  if (screen != nullptr) XRRFreeScreenResources(screen);
  XCloseDisplay(display);
  return topology;
}

#endif  // PCV_HAVE_XRANDR

void applyGpuPreference(GpuPreference preference, const PrimeTopology& topology) {
  if (preference == GpuPreference::Integrated) {
    logInfo("rendering on the default GPU (--gpu intel)");
    return;
  }

  if (preference == GpuPreference::Auto && topology.reversePrimeActive) {
    // The display is already being fed across the bus. Adding offload on
    // top would send every frame discrete -> integrated -> discrete, which
    // is how a maximised window on a 2560x1440 monitor ends up stalling
    // the compositor rather than merely running slowly.
    logInfo("'{}' is driving a display through reverse PRIME, so frames are already "
            "copied between GPUs for scanout; not forcing render offload",
            topology.sinkProviderName.empty() ? "a sink provider" : topology.sinkProviderName);
    logInfo("pass --gpu nvidia to force the discrete GPU anyway");
    return;
  }

  ::setenv("__NV_PRIME_RENDER_OFFLOAD", "1", 1);
  ::setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", 1);
  // The Vulkan equivalent: harmless here, and correct if a Vulkan backend
  // is ever added.
  ::setenv("__VK_LAYER_NV_optimus", "NVIDIA_only", 1);

  if (preference == GpuPreference::Discrete && topology.reversePrimeActive)
    logWarn("forcing render offload while '{}' drives a display through reverse PRIME; "
            "every frame will cross the bus twice and large windows may stutter badly",
            topology.sinkProviderName);
}

}  // namespace pcv
