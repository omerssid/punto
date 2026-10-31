// LAZ point decompression, behind laszip.
//
// Kept separate from las_loader.cpp so that file stays free of conditional
// compilation: the LAS reader calls decodeLazPoints unconditionally, and
// the stub reports the missing dependency when laszip was not found at
// configure time, or could not be loaded at runtime.
#pragma once

#include <cstddef>
#include <cstdint>
#include <stop_token>
#include <string>
#include <vector>

#include "core/large_buffer.hpp"
#include "io/cloud_builder.hpp"

namespace punto {

// True when LAZ can be decompressed: the build found laszip's header, and
// the library itself loads at runtime.
bool lazSupported();

// Why lazSupported() is false, worded for the user and naming what to
// install. Empty when it is true.
std::string lazUnavailableReason();

// Decompresses the point records of `path` into `records`, laid out exactly
// as the equivalent uncompressed LAS file would store them, so the same
// decode plan serves both.
//
// `fields` is the layout produced for the file's point data record format,
// and `stride` the natural packed size of those fields.
//
// `records` is a LargeBuffer rather than a vector because every byte of it
// is overwritten by the decode: a vector would first zero the whole
// allocation, which on an 87M-point file is 3.1 GB written twice for no
// reason, and would fault it in 4 KB at a time instead of in huge pages.
bool decodeLazPoints(const std::string& path, const std::vector<SourceField>& fields,
                     std::size_t stride, std::uint8_t pointFormat, std::size_t pointCount,
                     unsigned workers, std::stop_token stop, LargeBuffer<std::byte>& records,
                     std::vector<std::string>& warnings, std::string& error);

}  // namespace punto
