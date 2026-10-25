#include "io/laz_decode.hpp"

#include <atomic>
#include <mutex>
#include <cstring>

#include "core/log.hpp"
#include "core/parallel.hpp"

#ifdef PUNTO_HAVE_LASZIP
#include <laszip/laszip_api.h>
#endif

namespace punto {

#ifndef PUNTO_HAVE_LASZIP

bool lazSupported() { return false; }

bool decodeLazPoints(const std::string&, const std::vector<SourceField>&, std::size_t,
                     std::uint8_t, std::size_t, unsigned, std::stop_token,
                     std::vector<std::byte>&, std::vector<std::string>&, std::string& error) {
  error =
      "LAZ files are compressed and this build has no laszip. Install liblaszip-dev "
      "and re-run cmake.";
  return false;
}

#else

namespace {

// Writes one value into the record at a field's offset.
template <typename T>
void put(std::byte* record, std::size_t offset, T value) {
  std::memcpy(record + offset, &value, sizeof(T));
}

// Byte offsets of every field we fill, resolved once from the layout so the
// inner loop does no string comparison.
struct RecordOffsets {
  std::size_t x = 0, y = 0, z = 0;
  std::size_t intensity = 0;
  std::size_t returnInfo = 0;
  std::size_t classFlags = 0;
  std::size_t classification = 0;
  std::size_t userData = 0;
  std::size_t scanAngle = 0;
  std::size_t pointSourceId = 0;
  std::size_t gpsTime = 0;
  std::size_t red = 0, green = 0, blue = 0, nir = 0;

  bool hasClassFlags = false;
  bool hasGpsTime = false;
  bool hasColor = false;
  bool hasNir = false;
};

RecordOffsets resolveOffsets(const std::vector<SourceField>& fields) {
  RecordOffsets o;
  for (const SourceField& f : fields) {
    if (f.name == "x") o.x = f.byteOffset;
    else if (f.name == "y") o.y = f.byteOffset;
    else if (f.name == "z") o.z = f.byteOffset;
    else if (f.name == "intensity") o.intensity = f.byteOffset;
    else if (f.name == "return_info") o.returnInfo = f.byteOffset;
    else if (f.name == "class_flags") { o.classFlags = f.byteOffset; o.hasClassFlags = true; }
    else if (f.name == "classification") o.classification = f.byteOffset;
    else if (f.name == "user_data") o.userData = f.byteOffset;
    else if (f.name == "scan_angle") o.scanAngle = f.byteOffset;
    else if (f.name == "point_source_id") o.pointSourceId = f.byteOffset;
    else if (f.name == "gps_time") { o.gpsTime = f.byteOffset; o.hasGpsTime = true; }
    else if (f.name == "red") { o.red = f.byteOffset; o.hasColor = true; }
    else if (f.name == "green") o.green = f.byteOffset;
    else if (f.name == "blue") o.blue = f.byteOffset;
    else if (f.name == "nir") { o.nir = f.byteOffset; o.hasNir = true; }
  }
  return o;
}

// Serialises one decompressed point back into the on-disk LAS record
// layout.
//
// Round-tripping through the file format rather than writing straight into
// the cloud is deliberate: LAZ then flows through exactly the same decode
// plan as LAS -- the same scale and offset, the same colour-depth
// heuristic, the same classification masking -- so the two cannot drift
// apart, and a test can demand they produce identical clouds.
void writeRecord(const laszip_point* point, const RecordOffsets& o, bool legacy,
                 std::byte* record) {
  put<std::int32_t>(record, o.x, point->X);
  put<std::int32_t>(record, o.y, point->Y);
  put<std::int32_t>(record, o.z, point->Z);
  put<std::uint16_t>(record, o.intensity, point->intensity);

  if (legacy) {
    const auto returnInfo = static_cast<std::uint8_t>(
        (point->return_number & 0x07u) | ((point->number_of_returns & 0x07u) << 3) |
        ((point->scan_direction_flag & 0x01u) << 6) |
        ((point->edge_of_flight_line & 0x01u) << 7));
    put<std::uint8_t>(record, o.returnInfo, returnInfo);

    // laszip splits the classification byte into a 5-bit class and three
    // flag bits. Reassembling it means the LAS reader's masking pass runs
    // on LAZ too, warning count and all.
    const auto classification = static_cast<std::uint8_t>(
        (point->classification & 0x1Fu) | ((point->synthetic_flag & 0x01u) << 5) |
        ((point->keypoint_flag & 0x01u) << 6) | ((point->withheld_flag & 0x01u) << 7));
    put<std::uint8_t>(record, o.classification, classification);

    put<std::int8_t>(record, o.scanAngle, point->scan_angle_rank);
  } else {
    const auto returnInfo = static_cast<std::uint8_t>(
        (point->extended_return_number & 0x0Fu) |
        ((point->extended_number_of_returns & 0x0Fu) << 4));
    put<std::uint8_t>(record, o.returnInfo, returnInfo);

    if (o.hasClassFlags) {
      const auto flags = static_cast<std::uint8_t>(
          (point->extended_classification_flags & 0x0Fu) |
          ((point->extended_scanner_channel & 0x03u) << 4) |
          ((point->scan_direction_flag & 0x01u) << 6) |
          ((point->edge_of_flight_line & 0x01u) << 7));
      put<std::uint8_t>(record, o.classFlags, flags);
    }
    // Formats 6-10 carry the full class in its own byte, with no flags
    // packed into it.
    put<std::uint8_t>(record, o.classification, point->extended_classification);
    put<std::int16_t>(record, o.scanAngle, point->extended_scan_angle);
  }

  put<std::uint8_t>(record, o.userData, point->user_data);
  put<std::uint16_t>(record, o.pointSourceId, point->point_source_ID);
  if (o.hasGpsTime) put<double>(record, o.gpsTime, point->gps_time);
  if (o.hasColor) {
    put<std::uint16_t>(record, o.red, point->rgb[0]);
    put<std::uint16_t>(record, o.green, point->rgb[1]);
    put<std::uint16_t>(record, o.blue, point->rgb[2]);
  }
  if (o.hasNir) put<std::uint16_t>(record, o.nir, point->rgb[3]);
}

std::string lastError(laszip_POINTER reader) {
  laszip_CHAR* message = nullptr;
  if (reader != nullptr && laszip_get_error(reader, &message) == 0 && message != nullptr)
    return message;
  return "unknown laszip error";
}

// One reader, closed and destroyed however the scope is left.
class LaszipReader {
 public:
  ~LaszipReader() { close(); }

  bool open(const std::string& path, std::string& error) {
    if (laszip_create(&handle_) != 0) {
      error = "laszip_create failed";
      return false;
    }
    laszip_BOOL compressed = 0;
    if (laszip_open_reader(handle_, path.c_str(), &compressed) != 0) {
      error = lastError(handle_);
      close();
      return false;
    }
    opened_ = true;
    if (laszip_get_point_pointer(handle_, &point_) != 0) {
      error = lastError(handle_);
      close();
      return false;
    }
    return true;
  }

  bool seek(std::size_t index, std::string& error) {
    if (index == 0) return true;
    if (laszip_seek_point(handle_, static_cast<laszip_I64>(index)) != 0) {
      error = lastError(handle_);
      return false;
    }
    return true;
  }

  bool read(std::string& error) {
    if (laszip_read_point(handle_) != 0) {
      error = lastError(handle_);
      return false;
    }
    return true;
  }

  const laszip_point* point() const { return point_; }

  void close() {
    if (handle_ == nullptr) return;
    if (opened_) laszip_close_reader(handle_);
    laszip_destroy(handle_);
    handle_ = nullptr;
    point_ = nullptr;
    opened_ = false;
  }

 private:
  laszip_POINTER handle_ = nullptr;
  laszip_point* point_ = nullptr;
  bool opened_ = false;
};

}  // namespace

bool lazSupported() { return true; }

bool decodeLazPoints(const std::string& path, const std::vector<SourceField>& fields,
                     std::size_t stride, std::uint8_t pointFormat, std::size_t pointCount,
                     unsigned workers, std::stop_token stop, LargeBuffer<std::byte>& records,
                     std::vector<std::string>& warnings, std::string& error) {
  if (workers == 0) workers = defaultWorkerCount();
  const RecordOffsets offsets = resolveOffsets(fields);
  const bool legacy = pointFormat <= 5;

  // No value-initialisation: the decode writes every byte, and zeroing
  // first would cost a second full pass over several gigabytes.
  records.allocate(stride * pointCount);

  // Each worker opens its own reader and seeks to the start of its range.
  //
  // LAZ is chunked, so a seek decompresses from the start of the containing
  // chunk -- typically 50,000 points. With one seek per worker that
  // overhead is trivial next to decompressing the whole file, and
  // decompression is exactly the CPU-bound work that wants every core.
  std::atomic<bool> failed{false};
  std::atomic<bool> cancelled{false};
  std::mutex errorMutex;
  std::string firstError;

  parallelRangesCancellable(
      pointCount, workers, std::move(stop),
      [&](std::size_t begin, std::size_t end, unsigned, const std::stop_token& token) {
        if (begin >= end || failed.load(std::memory_order_relaxed)) return;

        LaszipReader reader;
        std::string localError;
        auto fail = [&] {
          failed.store(true, std::memory_order_relaxed);
          std::lock_guard lock(errorMutex);
          if (firstError.empty()) firstError = localError;
        };

        if (!reader.open(path, localError)) { fail(); return; }
        if (!reader.seek(begin, localError)) { fail(); return; }

        for (std::size_t i = begin; i < end; ++i) {
          // Checked per block rather than per point: the atomic load would
          // otherwise rival the cost of decompressing a point.
          if ((i & 0xFFFFu) == 0) {
            if (token.stop_requested()) {
              cancelled.store(true, std::memory_order_relaxed);
              return;
            }
            if (failed.load(std::memory_order_relaxed)) return;
          }
          if (!reader.read(localError)) { fail(); return; }
          writeRecord(reader.point(), offsets, legacy, records.get() + i * stride);
        }
      });

  if (cancelled.load(std::memory_order_relaxed)) {
    error = "load cancelled";
    return false;
  }
  if (failed.load(std::memory_order_relaxed)) {
    error = "LAZ: " + (firstError.empty() ? std::string("decompression failed") : firstError);
    return false;
  }
  (void)warnings;
  return true;
}

#endif  // PUNTO_HAVE_LASZIP

}  // namespace punto
