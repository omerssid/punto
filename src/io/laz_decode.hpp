// LAZ point decompression, behind laszip.
//
// Kept separate from las_loader.cpp so that file stays free of conditional
// compilation: the LAS reader calls decodeLazPoints unconditionally, and
// the stub below reports the missing dependency when laszip was not found
// at configure time.
#pragma once

#include <cstddef>
#include <cstdint>
#include <stop_token>
#include <string>
#include <vector>

#include "io/cloud_builder.hpp"

namespace pcv {

// True when this build can decompress LAZ.
bool lazSupported();

// Decompresses the point records of `path` into `records`, laid out exactly
// as the equivalent uncompressed LAS file would store them, so the same
// decode plan serves both.
//
// `fields` is the layout produced for the file's point data record format,
// and `stride` the natural packed size of those fields.
bool decodeLazPoints(const std::string& path, const std::vector<SourceField>& fields,
                     std::size_t stride, std::uint8_t pointFormat, std::size_t pointCount,
                     unsigned workers, std::stop_token stop, std::vector<std::byte>& records,
                     std::vector<std::string>& warnings, std::string& error);

}  // namespace pcv
