#include "hexapod_hardware_cpp/klann_geometry.hpp"
#include "hexapod_hardware_cpp/path_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using hexapod_hardware_cpp::LegModel;
using hexapod_hardware_cpp::LinkageSolver;

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: klann_lut_generator <input_yaml> <output_dir> [resolution]\n";
    return 2;
  }

  const std::string input_yaml = argv[1];
  const fs::path output_dir = argv[2];
  const int resolution_override = (argc >= 4) ? std::stoi(argv[3]) : 131072;

  fs::create_directories(output_dir / "lut");

  std::vector<LegModel> models;
  try {
    models = LinkageSolver::load_models_from_yaml(input_yaml);
  } catch (const std::exception & ex) {
    std::cerr << "Failed to load models: " << ex.what() << "\n";
    return 1;
  }

  LinkageSolver solver;
  YAML::Node root = YAML::LoadFile(input_yaml);
  YAML::Node out_root;
  out_root["legs"] = YAML::Node(YAML::NodeType::Sequence);

  for (std::size_t i = 0; i < models.size(); ++i) {
    auto model = models[i];
    model.lookup_resolution = resolution_override;
    solver.generate_lookup_table(model);

    const fs::path lut_path = output_dir / "lut" / (model.name + "_17bit.klut");
    if (!LinkageSolver::save_lookup_table_binary(model, lut_path.string())) {
      std::cerr << "Failed to save LUT for " << model.name << " to " << lut_path << "\n";
      return 1;
    }

    YAML::Node leg = root["legs"][static_cast<int>(i)];
    leg["lookup_resolution"] = resolution_override;
    leg["lookup_file"] = (fs::path("lut") / (model.name + "_17bit.klut")).string();
    leg["lookup_generate_if_missing"] = false;
    leg["lookup_save_generated"] = false;
    leg["use_lookup_table"] = true;
    out_root["legs"].push_back(leg);

    std::cout << "Wrote " << lut_path << "\n";
  }

  const fs::path out_yaml = output_dir / "linkage_precomputed.yaml";
  std::ofstream yaml_out(out_yaml);
  yaml_out << out_root;
  yaml_out.close();
  std::cout << "Wrote patched YAML to " << out_yaml << "\n";
  return 0;
}
