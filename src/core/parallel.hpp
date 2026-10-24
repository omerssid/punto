// Fork-join parallelism over point ranges.
//
// Loading is the one place where we genuinely have 28 cores of embarrassingly
// parallel work, so this is deliberately simple: split [0, n) into one
// contiguous block per worker and join. Contiguous blocks (rather than a work
// queue) matter here because each worker then walks the mmap'd file linearly,
// which is what the kernel's readahead is tuned for.
#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <stop_token>
#include <thread>
#include <vector>

namespace punto {

// Number of workers to use by default. Honours PUNTO_THREADS if set.
unsigned defaultWorkerCount();

// Calls body(begin, end, workerIndex) once per worker on disjoint
// sub-ranges covering [0, n). Runs inline without spawning when n is small
// or only one worker is requested.
void parallelRanges(std::size_t n, unsigned workers,
                    const std::function<void(std::size_t, std::size_t, unsigned)>& body);

inline void parallelRanges(std::size_t n,
                           const std::function<void(std::size_t, std::size_t, unsigned)>& body) {
  parallelRanges(n, defaultWorkerCount(), body);
}

// Cancellable variant: workers observe `stop` and may return early. Callers
// must treat partially-filled output as invalid when stop_requested() is set.
void parallelRangesCancellable(
    std::size_t n, unsigned workers, std::stop_token stop,
    const std::function<void(std::size_t, std::size_t, unsigned, const std::stop_token&)>& body);

}  // namespace punto
