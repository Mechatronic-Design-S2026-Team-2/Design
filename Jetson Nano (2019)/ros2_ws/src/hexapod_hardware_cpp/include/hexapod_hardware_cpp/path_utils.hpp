#ifndef HEXAPOD_HARDWARE_CPP__PATH_UTILS_HPP_
#define HEXAPOD_HARDWARE_CPP__PATH_UTILS_HPP_

#include <string>

namespace hexapod_hardware_cpp
{
std::string expand_user_path(const std::string & path);
std::string workspace_root_from_package_prefix(const std::string & package_name);
std::string default_generated_linkage_yaml(const std::string & package_name);
std::string default_measured_linkage_yaml(const std::string & package_name);
std::string auto_linkage_yaml(const std::string & package_name);
}

#endif
