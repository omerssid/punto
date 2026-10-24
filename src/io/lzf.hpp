// LZF decompression, as used by PCD's `DATA binary_compressed`.
//
// PCL compresses with Marc Lehmann's liblzf. The format is tiny -- a stream of
// literal runs and back-references -- so decoding it here avoids taking a
// dependency just to open a compressed PCD.
#pragma once

#include <cstddef>
#include <cstdint>

namespace pcv {

// Decompresses `inLen` bytes at `in` into the `outLen`-byte buffer at `out`.
// Returns the number of bytes written, or 0 if the stream is malformed or
// would overrun the output buffer.
std::size_t lzfDecompress(const std::uint8_t* in, std::size_t inLen, std::uint8_t* out,
                          std::size_t outLen);

}  // namespace pcv
