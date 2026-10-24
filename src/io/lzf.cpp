#include "io/lzf.hpp"

namespace pcv {

// Control-byte encoding:
//   ctrl < 32              -> literal run of (ctrl + 1) bytes follows
//   ctrl >= 32             -> back-reference; high 3 bits are length-2,
//                             low 5 bits are the high bits of the offset.
//                             A length nibble of 7 means an extra length
//                             byte follows. One more byte carries the low
//                             8 bits of the offset.
std::size_t lzfDecompress(const std::uint8_t* in, std::size_t inLen, std::uint8_t* out,
                          std::size_t outLen) {
  std::size_t ip = 0;  // input cursor
  std::size_t op = 0;  // output cursor

  while (ip < inLen) {
    std::uint32_t ctrl = in[ip++];

    if (ctrl < (1 << 5)) {
      // Literal run.
      const std::size_t run = ctrl + 1;
      if (op + run > outLen || ip + run > inLen) return 0;
      for (std::size_t i = 0; i < run; ++i) out[op++] = in[ip++];
      continue;
    }

    // Back-reference.
    std::size_t len = ctrl >> 5;
    if (len == 7) {
      if (ip >= inLen) return 0;
      len += in[ip++];
    }
    if (ip >= inLen) return 0;

    // Offset is "distance back from the current output position, minus 1".
    const std::size_t offset = ((ctrl & 0x1f) << 8) | in[ip++];
    const std::size_t distance = offset + 1;
    if (distance > op) return 0;  // reference before the start of output

    // len was stored as length-2.
    const std::size_t copy = len + 2;
    if (op + copy > outLen) return 0;

    // Must copy byte-by-byte: overlapping runs (distance < copy) are how LZF
    // encodes repeats, so memcpy/memmove would give the wrong result.
    std::size_t ref = op - distance;
    for (std::size_t i = 0; i < copy; ++i) out[op++] = out[ref++];
  }

  return op;
}

}  // namespace pcv
