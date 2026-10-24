#include "io/mapped_file.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace punto {

MappedFile::~MappedFile() { close(); }

MappedFile::MappedFile(MappedFile&& other) noexcept
    : data_(std::exchange(other.data_, nullptr)), size_(std::exchange(other.size_, 0)) {}

MappedFile& MappedFile::operator=(MappedFile&& other) noexcept {
  if (this != &other) {
    close();
    data_ = std::exchange(other.data_, nullptr);
    size_ = std::exchange(other.size_, 0);
  }
  return *this;
}

bool MappedFile::open(const std::string& path, std::string& error) {
  close();

  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    error = "cannot open '" + path + "': " + std::strerror(errno);
    return false;
  }

  struct stat st {};
  if (::fstat(fd, &st) != 0) {
    error = "cannot stat '" + path + "': " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  if (!S_ISREG(st.st_mode)) {
    error = "'" + path + "' is not a regular file";
    ::close(fd);
    return false;
  }
  if (st.st_size == 0) {
    error = "'" + path + "' is empty";
    ::close(fd);
    return false;
  }

  void* addr = ::mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ, MAP_PRIVATE, fd, 0);
  // The mapping keeps its own reference to the file, so the descriptor is
  // no longer needed once mmap succeeds.
  ::close(fd);

  if (addr == MAP_FAILED) {
    error = "cannot map '" + path + "': " + std::strerror(errno);
    return false;
  }

  data_ = addr;
  size_ = static_cast<std::size_t>(st.st_size);
  return true;
}

void MappedFile::close() {
  if (data_ != nullptr) {
    ::munmap(data_, size_);
    data_ = nullptr;
    size_ = 0;
  }
}

void MappedFile::adviseSequential(std::size_t offset, std::size_t length) const {
  if (data_ == nullptr || offset >= size_) return;
  length = std::min(length, size_ - offset);
  // Page-align the start; madvise requires it.
  const std::size_t pageSize = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  const std::size_t alignedOffset = offset & ~(pageSize - 1);
  length += offset - alignedOffset;
  auto* base = static_cast<char*>(data_) + alignedOffset;
  ::madvise(base, length, MADV_SEQUENTIAL);
  ::madvise(base, length, MADV_WILLNEED);
}

}  // namespace punto
