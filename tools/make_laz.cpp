// las2laz -- compress a LAS fixture with laszip itself.
//
// Built only when laszip is available. Using the reference implementation
// to produce the test data means the LAZ reader is checked against what
// real files look like, not against our own idea of the format.
#include <cstdio>
#include <cstring>
#include <string>

#include <laszip/laszip_api.h>

namespace {

std::string errorOf(laszip_POINTER handle) {
  laszip_CHAR* message = nullptr;
  if (handle != nullptr && laszip_get_error(handle, &message) == 0 && message != nullptr)
    return message;
  return "unknown laszip error";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::printf("usage: pcv_make_laz <in.las> <out.laz>\n");
    return 2;
  }
  const std::string input = argv[1];
  const std::string output = argv[2];

  laszip_POINTER reader = nullptr;
  laszip_POINTER writer = nullptr;
  if (laszip_create(&reader) != 0 || laszip_create(&writer) != 0) {
    std::fprintf(stderr, "laszip_create failed\n");
    return 1;
  }

  laszip_BOOL compressed = 0;
  if (laszip_open_reader(reader, input.c_str(), &compressed) != 0) {
    std::fprintf(stderr, "open %s: %s\n", input.c_str(), errorOf(reader).c_str());
    return 1;
  }

  laszip_header* header = nullptr;
  laszip_point* point = nullptr;
  if (laszip_get_header_pointer(reader, &header) != 0 ||
      laszip_get_point_pointer(reader, &point) != 0) {
    std::fprintf(stderr, "header/point: %s\n", errorOf(reader).c_str());
    return 1;
  }

  const laszip_I64 count = static_cast<laszip_I64>(
      header->number_of_point_records != 0 ? header->number_of_point_records
                                           : header->extended_number_of_point_records);

  if (laszip_set_header(writer, header) != 0) {
    std::fprintf(stderr, "set_header: %s\n", errorOf(writer).c_str());
    return 1;
  }
  // A small chunk size, so even a 5000-point fixture spans several chunks
  // and the reader's per-worker seeking is actually exercised.
  laszip_set_chunk_size(writer, 1024);
  if (laszip_open_writer(writer, output.c_str(), 1) != 0) {
    std::fprintf(stderr, "open %s: %s\n", output.c_str(), errorOf(writer).c_str());
    return 1;
  }

  laszip_point* outPoint = nullptr;
  if (laszip_get_point_pointer(writer, &outPoint) != 0) {
    std::fprintf(stderr, "writer point: %s\n", errorOf(writer).c_str());
    return 1;
  }

  for (laszip_I64 i = 0; i < count; ++i) {
    if (laszip_read_point(reader) != 0) {
      std::fprintf(stderr, "read point %lld: %s\n", static_cast<long long>(i),
                   errorOf(reader).c_str());
      return 1;
    }
    if (laszip_set_point(writer, point) != 0 || laszip_write_point(writer) != 0) {
      std::fprintf(stderr, "write point %lld: %s\n", static_cast<long long>(i),
                   errorOf(writer).c_str());
      return 1;
    }
  }

  laszip_close_writer(writer);
  laszip_close_reader(reader);
  laszip_destroy(writer);
  laszip_destroy(reader);
  std::printf("  wrote %s (%lld points)\n", output.c_str(), static_cast<long long>(count));
  return 0;
}
