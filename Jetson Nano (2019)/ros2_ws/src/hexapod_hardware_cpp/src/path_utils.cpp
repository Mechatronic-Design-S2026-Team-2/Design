#include "hexapod_hardware_cpp/path_utils.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <cstdlib>
#include <filesystem>

namespace hexapod_hardware_cpp
{
std::string expand_user_path(const std::string & path)
{
  if (path.empty()) return path;
  if (path[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      if (path.size() == 1) return std::string(home);
      if (path[1] == '/') return std::string(home) + path.substr(1);
    }
  }
  return path;
}

std::string workspace_root_from_package_prefix(const std::string & package_name)
{
  namespace fs = std::filesystem;
  const fs::path prefix(ament_index_cpp::get_package_prefix(package_name));
  return prefix.parent_path().parent_path().string();
}

std::string default_generated_linkage_yaml(const std::string & package_name)
{
  namespace fs = std::filesystem;
  return (fs::path(workspace_root_from_package_prefix(package_name)) / "hexapod_lut_out" / "linkage_precomputed.yaml").string();
}

std::string default_measured_linkage_yaml(const std::string & package_name)
{
  namespace fs = std::filesystem;
  const fs::path prefix(ament_index_cpp::get_package_prefix(package_name));
  return (prefix / "share" / package_name / "config" / "linkage_measured.yaml").string();
}

std::string auto_linkage_yaml(const std::string & package_name)
{
  namespace fs = std::filesystem;
  const auto generated = default_generated_linkage_yaml(package_name);
  if (fs::exists(generated)) {
    return generated;
  }
  return default_measured_linkage_yaml(package_name);
}
}  // namespace hexapod_hardware_cpp
