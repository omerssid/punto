#include "io/loader.hpp"

namespace pcv {

LoadResult loadPly(const std::string&, const LoadOptions&, PointCloud&, std::stop_token) {
  return LoadResult::failure("PLY support not yet implemented");
}

}  // namespace pcv
