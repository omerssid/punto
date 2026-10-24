// Parallel LSD radix sort over 64-bit keys.
//
// Used to put points in Morton order. A comparison sort would do, but a
// radix sort over a known bit range is memory-bound rather than
// compare-bound, which is what matters at tens of millions of elements.
#pragma once

#include <cstdint>

#include "core/large_buffer.hpp"

namespace pcv {

// Sorts keys[0, count) by the `bitCount` bits starting at `firstBit`.
//
// `scratch` must hold at least `count` elements; the sort ping-pongs
// between the two buffers and copies back if it lands in the scratch one.
// Restricting the sort to the bits that matter is the point: Morton keys
// carry a payload in their low 32 bits whose order is irrelevant, so those
// passes are simply skipped.
void radixSortByBits(std::uint64_t* keys, std::uint64_t* scratch, std::size_t count,
                     int firstBit, int bitCount, unsigned workers = 0);

}  // namespace pcv
