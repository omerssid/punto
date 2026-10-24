#include "gpu/shader.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

#include "core/log.hpp"

namespace pcv {
namespace {

std::string readWholeFile(const std::filesystem::path& path, bool& ok) {
  std::ifstream in(path, std::ios::binary);
  if (!in) { ok = false; return {}; }
  std::ostringstream ss;
  ss << in.rdbuf();
  ok = true;
  return ss.str();
}

// Inserts `defines` directly after the #version line. They must come before
// any other statement, and #version must be the first thing in the source.
std::string injectDefines(const std::string& source, const std::vector<std::string>& defines) {
  if (defines.empty()) return source;

  std::size_t insertAt = 0;
  const std::size_t versionPos = source.find("#version");
  if (versionPos != std::string::npos) {
    const std::size_t eol = source.find('\n', versionPos);
    insertAt = (eol == std::string::npos) ? source.size() : eol + 1;
  }

  std::string block;
  for (const std::string& d : defines) block += "#define " + d + "\n";
  // Keep reported line numbers pointing at the real source.
  block += "#line 2\n";

  std::string out = source;
  out.insert(insertAt, block);
  return out;
}

bool compileStage(GLenum stage, const std::string& source, const char* label, GLuint& outShader,
                  std::string& error) {
  outShader = glCreateShader(stage);
  const char* src = source.c_str();
  glShaderSource(outShader, 1, &src, nullptr);
  glCompileShader(outShader);

  GLint ok = GL_FALSE;
  glGetShaderiv(outShader, GL_COMPILE_STATUS, &ok);
  if (ok == GL_TRUE) return true;

  GLint logLength = 0;
  glGetShaderiv(outShader, GL_INFO_LOG_LENGTH, &logLength);
  std::string log(static_cast<std::size_t>(std::max(logLength, 1)), '\0');
  glGetShaderInfoLog(outShader, logLength, nullptr, log.data());
  error = std::string(label) + ": " + log;
  glDeleteShader(outShader);
  outShader = 0;
  return false;
}

bool linkProgram(GLuint program, std::string& error) {
  glLinkProgram(program);
  GLint ok = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &ok);
  if (ok == GL_TRUE) return true;

  GLint logLength = 0;
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
  std::string log(static_cast<std::size_t>(std::max(logLength, 1)), '\0');
  glGetProgramInfoLog(program, logLength, nullptr, log.data());
  error = "link: " + log;
  return false;
}

long long fileTimestamp(const std::filesystem::path& path) {
  std::error_code ec;
  const auto t = std::filesystem::last_write_time(path, ec);
  if (ec) return 0;
  return t.time_since_epoch().count();
}

}  // namespace

// ------------------------------------------------------------ ShaderLibrary

ShaderLibrary& ShaderLibrary::instance() {
  static ShaderLibrary library;
  return library;
}

ShaderLibrary::ShaderLibrary() {
  // An explicit override wins; otherwise look for the source tree's shader
  // directory, which is present during development and absent when the
  // binary is installed elsewhere.
  if (const char* env = std::getenv("PCV_SHADER_DIR")) {
    if (std::filesystem::is_directory(env)) {
      diskDir_ = env;
    } else {
      logWarn("PCV_SHADER_DIR='{}' is not a directory; using built-in shaders", env);
    }
  }
#ifdef PCV_SOURCE_SHADER_DIR
  if (diskDir_.empty() && std::filesystem::is_directory(PCV_SOURCE_SHADER_DIR))
    diskDir_ = PCV_SOURCE_SHADER_DIR;
#endif
  if (!diskDir_.empty()) logInfo("shader live reload from {}", diskDir_);
}

void ShaderLibrary::registerBuiltin(std::string name, std::string source) {
  builtins_.emplace(std::move(name), std::move(source));
}

bool ShaderLibrary::readSource(const std::string& name, std::string& out, std::string& error) {
  if (!diskDir_.empty()) {
    const std::filesystem::path path = std::filesystem::path(diskDir_) / name;
    if (std::filesystem::exists(path)) {
      bool ok = false;
      out = readWholeFile(path, ok);
      if (ok) {
        diskTimestamps_[name] = fileTimestamp(path);
        return true;
      }
      error = "cannot read " + path.string();
      return false;
    }
  }
  const auto it = builtins_.find(name);
  if (it == builtins_.end()) {
    error = "no shader named '" + name + "'";
    return false;
  }
  out = it->second;
  return true;
}

bool ShaderLibrary::expandIncludes(const std::string& name, const std::string& source,
                                   std::string& out, std::vector<std::string>& stack,
                                   std::string& error) {
  for (const std::string& s : stack) {
    if (s == name) {
      error = "circular #include of '" + name + "'";
      return false;
    }
  }
  stack.push_back(name);

  std::istringstream in(source);
  std::string line;
  while (std::getline(in, line)) {
    const std::size_t hash = line.find_first_not_of(" \t");
    const bool isInclude = hash != std::string::npos &&
                           line.compare(hash, 8, "#include") == 0;
    if (!isInclude) {
      out += line;
      out += '\n';
      continue;
    }
    const std::size_t open = line.find('"', hash);
    const std::size_t close = open == std::string::npos ? std::string::npos : line.find('"', open + 1);
    if (open == std::string::npos || close == std::string::npos) {
      error = name + ": malformed #include";
      return false;
    }
    const std::string included = line.substr(open + 1, close - open - 1);
    std::string includedSource;
    if (!readSource(included, includedSource, error)) return false;
    if (!expandIncludes(included, includedSource, out, stack, error)) return false;
  }

  stack.pop_back();
  return true;
}

bool ShaderLibrary::resolve(const std::string& name, std::string& out, std::string& error) {
  std::string source;
  if (!readSource(name, source, error)) return false;
  out.clear();
  std::vector<std::string> stack;
  return expandIncludes(name, source, out, stack, error);
}

bool ShaderLibrary::pollForChanges() {
  if (diskDir_.empty()) return false;
  bool changed = false;
  for (auto& [name, stamp] : diskTimestamps_) {
    const std::filesystem::path path = std::filesystem::path(diskDir_) / name;
    const long long now = fileTimestamp(path);
    if (now != 0 && now != stamp) {
      stamp = now;
      changed = true;
    }
  }
  return changed;
}

// ----------------------------------------------------------- ShaderProgram

ShaderProgram::~ShaderProgram() { destroy(); }

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept
    : program_(std::exchange(other.program_, 0)),
      locations_(std::move(other.locations_)),
      isCompute_(other.isCompute_),
      vertexName_(std::move(other.vertexName_)),
      fragmentName_(std::move(other.fragmentName_)),
      computeName_(std::move(other.computeName_)),
      defines_(std::move(other.defines_)) {}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
  if (this != &other) {
    destroy();
    program_ = std::exchange(other.program_, 0);
    locations_ = std::move(other.locations_);
    isCompute_ = other.isCompute_;
    vertexName_ = std::move(other.vertexName_);
    fragmentName_ = std::move(other.fragmentName_);
    computeName_ = std::move(other.computeName_);
    defines_ = std::move(other.defines_);
  }
  return *this;
}

void ShaderProgram::destroy() {
  if (program_ != 0) {
    glDeleteProgram(program_);
    program_ = 0;
  }
  locations_.clear();
}

bool ShaderProgram::buildGraphics(const std::string& vertexName, const std::string& fragmentName,
                                  const std::vector<std::string>& defines, std::string& error) {
  vertexName_ = vertexName;
  fragmentName_ = fragmentName;
  defines_ = defines;
  isCompute_ = false;
  return reload(error);
}

bool ShaderProgram::buildCompute(const std::string& computeName,
                                 const std::vector<std::string>& defines, std::string& error) {
  computeName_ = computeName;
  defines_ = defines;
  isCompute_ = true;
  return reload(error);
}

bool ShaderProgram::reload(std::string& error) {
  ShaderLibrary& library = ShaderLibrary::instance();
  std::vector<GLuint> stages;
  GLuint program = glCreateProgram();

  auto cleanup = [&](bool keepProgram) {
    for (GLuint s : stages) {
      if (!keepProgram) glDetachShader(program, s);
      glDeleteShader(s);
    }
    if (!keepProgram) glDeleteProgram(program);
  };

  auto addStage = [&](GLenum type, const std::string& name) -> bool {
    std::string source;
    if (!library.resolve(name, source, error)) return false;
    GLuint shader = 0;
    if (!compileStage(type, injectDefines(source, defines_), name.c_str(), shader, error))
      return false;
    glAttachShader(program, shader);
    stages.push_back(shader);
    return true;
  };

  bool ok = true;
  if (isCompute_) {
    ok = addStage(GL_COMPUTE_SHADER, computeName_);
  } else {
    ok = addStage(GL_VERTEX_SHADER, vertexName_) && addStage(GL_FRAGMENT_SHADER, fragmentName_);
  }
  if (!ok || !linkProgram(program, error)) {
    cleanup(false);
    // Deliberately leaves any previously linked program in place, so a typo
    // during live editing shows an error instead of blanking the viewport.
    return false;
  }

  cleanup(true);
  destroy();
  program_ = program;
  return true;
}

GLint ShaderProgram::location(const char* name) {
  const auto it = locations_.find(name);
  if (it != locations_.end()) return it->second;
  const GLint loc = glGetUniformLocation(program_, name);
  locations_.emplace(name, loc);
  return loc;
}

void ShaderProgram::set(const char* name, int value) {
  const GLint l = location(name);
  if (l >= 0) glUniform1i(l, value);
}
void ShaderProgram::set(const char* name, unsigned value) {
  const GLint l = location(name);
  if (l >= 0) glUniform1ui(l, value);
}
void ShaderProgram::set(const char* name, float value) {
  const GLint l = location(name);
  if (l >= 0) glUniform1f(l, value);
}
void ShaderProgram::set(const char* name, float x, float y) {
  const GLint l = location(name);
  if (l >= 0) glUniform2f(l, x, y);
}
void ShaderProgram::set(const char* name, float x, float y, float z) {
  const GLint l = location(name);
  if (l >= 0) glUniform3f(l, x, y, z);
}
void ShaderProgram::set(const char* name, float x, float y, float z, float w) {
  const GLint l = location(name);
  if (l >= 0) glUniform4f(l, x, y, z, w);
}
void ShaderProgram::set(const char* name, const Vec3f& v) { set(name, v.x, v.y, v.z); }
void ShaderProgram::set(const char* name, const Mat4& m) {
  const GLint l = location(name);
  if (l >= 0) glUniformMatrix4fv(l, 1, GL_FALSE, m.data());
}

}  // namespace pcv
