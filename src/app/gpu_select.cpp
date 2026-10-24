#include "app/gpu_select.hpp"

#include <cstdlib>

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

void applyGpuPreference(GpuPreference preference) {
  if (preference == GpuPreference::Integrated) {
    // Leave the system default alone; it is already the integrated GPU on
    // the hybrid setups where this matters.
    return;
  }
  // Auto and Discrete both request offload. If no NVIDIA driver is present
  // libglvnd falls back to the default vendor on its own, so Auto costs
  // nothing on machines without one.
  ::setenv("__NV_PRIME_RENDER_OFFLOAD", "1", 1);
  ::setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", 1);
  // The Vulkan equivalent: harmless here, and correct if a Vulkan backend
  // is ever added.
  ::setenv("__VK_LAYER_NV_optimus", "NVIDIA_only", 1);
}

}  // namespace pcv
