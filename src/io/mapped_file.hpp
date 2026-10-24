// Read-only memory mapping of a file.
//
// Loaders parse straight out of the mapping rather than read()-ing into a
// buffer: it avoids one full copy of (for a big HD map) half a gigabyte, and
// it lets every worker thread walk its own slice of the file with the kernel
// faulting pages in behind them.
#pragma once

#include <cstddef>
#include <span>
#include <string>
#include <string_view>

namespace punto {

class MappedFile {
 public:
  MappedFile() = default;
  ~MappedFile();

  MappedFile(const MappedFile&) = delete;
  MappedFile& operator=(const MappedFile&) = delete;
  MappedFile(MappedFile&& other) noexcept;
  MappedFile& operator=(MappedFile&& other) noexcept;

  bool open(const std::string& path, std::string& error);
  void close();

  bool isOpen() const { return data_ != nullptr; }
  std::size_t size() const { return size_; }

  std::span<const std::byte> bytes() const {
    return {static_cast<const std::byte*>(data_), size_};
  }
  const char* chars() const { return static_cast<const char*>(data_); }
  std::string_view view() const { return {chars(), size_}; }

  // Hints the kernel that `length` bytes from `offset` are about to be read
  // sequentially. Called once the header tells us where the payload starts.
  void adviseSequential(std::size_t offset, std::size_t length) const;

 private:
  void* data_ = nullptr;
  std::size_t size_ = 0;
};

}  // namespace punto
