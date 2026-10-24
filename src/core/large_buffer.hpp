// Allocator for the multi-hundred-megabyte scratch buffers the chunk builder
// and GPU upload path need.
//
// Two things ordinary containers get wrong at this size:
//
//  - std::vector value-initialises, so a gigabyte of scratch is zeroed and
//    then immediately overwritten.
//  - Fresh anonymous memory is faulted in a page at a time. At 4 KB that is
//    ~262,000 faults per gigabyte; this system has transparent huge pages set
//    to `madvise`, so asking for them explicitly turns that into ~512 faults.
//
// Together those two measured at roughly 300 ms per gigabyte on a 32M-point
// cloud -- more than any real work the partition was doing.
#pragma once

#include <sys/mman.h>

#include <cstddef>
#include <new>
#include <utility>

namespace pcv {

template <typename T>
class LargeBuffer {
  static_assert(std::is_trivially_destructible_v<T>,
                "LargeBuffer does not run destructors");

 public:
  LargeBuffer() = default;

  explicit LargeBuffer(std::size_t count) { allocate(count); }

  ~LargeBuffer() { release(); }

  LargeBuffer(const LargeBuffer&) = delete;
  LargeBuffer& operator=(const LargeBuffer&) = delete;

  LargeBuffer(LargeBuffer&& other) noexcept
      : data_(std::exchange(other.data_, nullptr)),
        count_(std::exchange(other.count_, 0)),
        bytes_(std::exchange(other.bytes_, 0)) {}

  LargeBuffer& operator=(LargeBuffer&& other) noexcept {
    if (this != &other) {
      release();
      data_ = std::exchange(other.data_, nullptr);
      count_ = std::exchange(other.count_, 0);
      bytes_ = std::exchange(other.bytes_, 0);
    }
    return *this;
  }

  void allocate(std::size_t count) {
    release();
    if (count == 0) return;
    const std::size_t bytes = count * sizeof(T);

    void* addr = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (addr == MAP_FAILED) throw std::bad_alloc();

    // Ask for 2 MB pages. Advisory: the kernel may decline, in which case
    // this is simply a normal mapping and nothing breaks.
    ::madvise(addr, bytes, MADV_HUGEPAGE);

    data_ = static_cast<T*>(addr);
    count_ = count;
    bytes_ = bytes;
  }

  void release() {
    if (data_ != nullptr) {
      ::munmap(data_, bytes_);
      data_ = nullptr;
      count_ = 0;
      bytes_ = 0;
    }
  }

  T* get() { return data_; }
  const T* get() const { return data_; }
  T& operator[](std::size_t i) { return data_[i]; }
  const T& operator[](std::size_t i) const { return data_[i]; }
  std::size_t size() const { return count_; }
  bool empty() const { return count_ == 0; }

 private:
  T* data_ = nullptr;
  std::size_t count_ = 0;
  std::size_t bytes_ = 0;
};

}  // namespace pcv
