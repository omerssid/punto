// RAII wrappers for GL buffer and texture objects.
#pragma once

#include <cstddef>
#include <span>
#include <utility>

#include "gpu/gl.hpp"

namespace punto {

class GlBuffer {
 public:
  GlBuffer() = default;
  ~GlBuffer() { destroy(); }

  GlBuffer(const GlBuffer&) = delete;
  GlBuffer& operator=(const GlBuffer&) = delete;
  GlBuffer(GlBuffer&& o) noexcept
      : id_(std::exchange(o.id_, 0)), bytes_(std::exchange(o.bytes_, 0)) {}
  GlBuffer& operator=(GlBuffer&& o) noexcept {
    if (this != &o) {
      destroy();
      id_ = std::exchange(o.id_, 0);
      bytes_ = std::exchange(o.bytes_, 0);
    }
    return *this;
  }

  // Allocates immutable storage. GL_DYNAMIC_STORAGE_BIT is required for
  // later glNamedBufferSubData; omit it for write-once data.
  void allocate(std::size_t bytes, GLbitfield flags = 0, const void* initial = nullptr) {
    destroy();
    if (bytes == 0) return;
    glCreateBuffers(1, &id_);
    glNamedBufferStorage(id_, static_cast<GLsizeiptr>(bytes), initial, flags);
    bytes_ = bytes;
  }

  void upload(std::size_t offset, std::size_t bytes, const void* data) {
    if (id_ == 0 || bytes == 0) return;
    glNamedBufferSubData(id_, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(bytes), data);
  }

  template <typename T>
  void uploadSpan(std::size_t elementOffset, std::span<const T> data) {
    upload(elementOffset * sizeof(T), data.size() * sizeof(T), data.data());
  }

  void download(std::size_t offset, std::size_t bytes, void* dst) const {
    if (id_ == 0 || bytes == 0) return;
    glGetNamedBufferSubData(id_, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(bytes), dst);
  }

  void clearToZero() const {
    if (id_ == 0) return;
    const GLuint zero = 0;
    glClearNamedBufferData(id_, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, &zero);
  }

  // Allocates a buffer the CPU can read without the driver migrating it
  // away from the GPU, and returns a pointer that stays valid for the
  // buffer's lifetime. Used for the per-frame statistics readback.
  const void* allocatePersistentRead(std::size_t bytes) {
    destroy();
    if (bytes == 0) return nullptr;
    const GLbitfield flags = GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;
    glCreateBuffers(1, &id_);
    glNamedBufferStorage(id_, static_cast<GLsizeiptr>(bytes), nullptr, flags);
    bytes_ = bytes;
    return glMapNamedBufferRange(id_, 0, static_cast<GLsizeiptr>(bytes), flags);
  }

  void copyFrom(const GlBuffer& src, std::size_t bytes) const {
    if (id_ == 0 || src.id_ == 0) return;
    glCopyNamedBufferSubData(src.id_, id_, 0, 0, static_cast<GLsizeiptr>(bytes));
  }

  void bindBase(GLenum target, GLuint index) const { glBindBufferBase(target, index, id_); }
  void bind(GLenum target) const { glBindBuffer(target, id_); }

  GLuint id() const { return id_; }
  std::size_t bytes() const { return bytes_; }
  bool valid() const { return id_ != 0; }

  void destroy() {
    if (id_ != 0) {
      glDeleteBuffers(1, &id_);
      id_ = 0;
      bytes_ = 0;
    }
  }

 private:
  GLuint id_ = 0;
  std::size_t bytes_ = 0;
};

class GlTexture {
 public:
  GlTexture() = default;
  ~GlTexture() { destroy(); }

  GlTexture(const GlTexture&) = delete;
  GlTexture& operator=(const GlTexture&) = delete;
  GlTexture(GlTexture&& o) noexcept : id_(std::exchange(o.id_, 0)) {}
  GlTexture& operator=(GlTexture&& o) noexcept {
    if (this != &o) {
      destroy();
      id_ = std::exchange(o.id_, 0);
    }
    return *this;
  }

  void create2D(int width, int height, GLenum internalFormat) {
    destroy();
    glCreateTextures(GL_TEXTURE_2D, 1, &id_);
    glTextureStorage2D(id_, 1, internalFormat, width, height);
    // Nearest and clamped: these are render targets read back per-pixel by
    // the post pass, never filtered or wrapped.
    glTextureParameteri(id_, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTextureParameteri(id_, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTextureParameteri(id_, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTextureParameteri(id_, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }

  void bindUnit(GLuint unit) const { glBindTextureUnit(unit, id_); }
  GLuint id() const { return id_; }
  bool valid() const { return id_ != 0; }

  void destroy() {
    if (id_ != 0) {
      glDeleteTextures(1, &id_);
      id_ = 0;
    }
  }

 private:
  GLuint id_ = 0;
};

// Matches GL's DrawArraysIndirectCommand layout exactly; the cull shader
// writes these and glMultiDrawArraysIndirect consumes them without any
// CPU-side involvement.
struct DrawArraysIndirectCommand {
  GLuint count;          // vertices to draw -- the LOD prefix length
  GLuint instanceCount;  // always 1
  GLuint first;          // chunk.firstPoint, so gl_VertexID is global
  GLuint baseInstance;   // chunk index, read back via gl_BaseInstanceARB
};
static_assert(sizeof(DrawArraysIndirectCommand) == 16);

}  // namespace punto
