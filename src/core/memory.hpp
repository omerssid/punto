// Host memory facts the OS can tell us, and the OOM-killer policy punto asks
// for.
#pragma once

#include <cstdint>

namespace punto {

// Memory the kernel believes can be allocated without pushing the system into
// swap, in bytes.
//
// This is MemAvailable rather than MemFree: it counts reclaimable page cache,
// which a large load does get to use. Returns 0 when the figure cannot be
// read, which callers must treat as "unknown" and not as "none".
std::uint64_t availableMemoryBytes();

// Total installed RAM in bytes, or 0 if unknown.
std::uint64_t totalMemoryBytes();

// Volunteers this process as the OOM killer's first victim.
//
// A viewer asking for several GB is far more expendable than the desktop
// session it runs inside. Left at the default, punto scores below a browser
// -- Chromium-family renderers raise their own oom_score_adj to 300 -- so an
// oversized open gets the browser killed and leaves the thrashing viewer
// running. Best effort; there is nothing useful to do if it fails.
void preferSelfForOomKill();

}  // namespace punto
