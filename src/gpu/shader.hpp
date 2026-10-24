// Shader program compilation, with #include resolution and live reload.
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "core/math.hpp"
#include "gpu/gl.hpp"

namespace pcv {

// Where shader sources come from.
//
// Sources are compiled into the binary so the viewer is a single file with
// nothing to install. When a shader directory is present (set via
// PCV_SHADER_DIR, or found next to the source tree during development) it
// takes priority, which is what makes live reload possible.
class ShaderLibrary {
 public:
  static ShaderLibrary& instance();

  // Registers a shader's built-in source. Called by the generated
  // embedded_shaders.cpp at static init.
  void registerBuiltin(std::string name, std::string source);

  // Returns the source for `name`, preferring an on-disk override.
  // Resolves #include "other.glsl" recursively.
  bool resolve(const std::string& name, std::string& out, std::string& error);

  // True when any on-disk shader changed since the last call.
  bool pollForChanges();

  const std::string& diskDirectory() const { return diskDir_; }
  bool liveReloadActive() const { return !diskDir_.empty(); }

 private:
  ShaderLibrary();
  bool readSource(const std::string& name, std::string& out, std::string& error);
  bool expandIncludes(const std::string& name, const std::string& source, std::string& out,
                      std::vector<std::string>& stack, std::string& error);

  std::unordered_map<std::string, std::string> builtins_;
  std::unordered_map<std::string, long long> diskTimestamps_;
  std::string diskDir_;
};

// A linked GL program. Owns its handle; moveable, not copyable.
class ShaderProgram {
 public:
  ShaderProgram() = default;
  ~ShaderProgram();
  ShaderProgram(const ShaderProgram&) = delete;
  ShaderProgram& operator=(const ShaderProgram&) = delete;
  ShaderProgram(ShaderProgram&& other) noexcept;
  ShaderProgram& operator=(ShaderProgram&& other) noexcept;

  // Builds from shader names resolved through ShaderLibrary. `defines` are
  // injected after the #version line.
  bool buildGraphics(const std::string& vertexName, const std::string& fragmentName,
                     const std::vector<std::string>& defines, std::string& error);
  bool buildCompute(const std::string& computeName, const std::vector<std::string>& defines,
                    std::string& error);

  // Recompiles from the names and defines last used. Leaves the existing
  // program intact if compilation fails, so a typo during live editing
  // does not blank the viewport.
  bool reload(std::string& error);

  GLuint id() const { return program_; }
  bool valid() const { return program_ != 0; }
  void use() const { glUseProgram(program_); }

  // Uniform setters. Locations are cached; an unknown name is ignored,
  // which keeps shader variants from needing matching C++ branches.
  void set(const char* name, int value);
  void set(const char* name, unsigned value);
  void set(const char* name, float value);
  void set(const char* name, float x, float y);
  void set(const char* name, float x, float y, float z);
  void set(const char* name, float x, float y, float z, float w);
  void set(const char* name, const Vec3f& v);
  void set(const char* name, const Mat4& m);

 private:
  void destroy();
  GLint location(const char* name);

  GLuint program_ = 0;
  std::unordered_map<std::string, GLint> locations_;

  // Retained for reload().
  bool isCompute_ = false;
  std::string vertexName_;
  std::string fragmentName_;
  std::string computeName_;
  std::vector<std::string> defines_;
};

}  // namespace pcv
