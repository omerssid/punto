// Discrete-GPU selection, before any GL call.
#pragma once

#include <string>

namespace pcv {

enum class GpuPreference { Auto, Discrete, Integrated };

bool parseGpuPreference(const std::string& text, GpuPreference& out);

// Must run before glfwInit().
//
// On a hybrid-graphics laptop the default GLX vendor is the integrated GPU,
// so a program that does nothing special renders on Intel even with an
// NVIDIA card present -- which is what pcl_viewer does here. libglvnd picks
// its vendor when the GL stack is first touched and reads these variables at
// that moment, so setting them in-process is enough and no wrapper script is
// needed. If there is no discrete GPU the variables are simply ignored.
void applyGpuPreference(GpuPreference preference);

}  // namespace pcv
