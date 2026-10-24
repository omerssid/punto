#include "io/loader.hpp"

namespace pcv {

LoadResult loadLas(const std::string&, const LoadOptions&, PointCloud&, std::stop_token) {
  return LoadResult::failure("LAS/LAZ support not yet implemented");
}

}  // namespace pcv
