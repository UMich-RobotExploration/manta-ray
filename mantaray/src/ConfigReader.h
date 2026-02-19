//
// Created by tko on 2/24/26.
//

#pragma once
#include <acoustics/Grid.h>
#include <json.hpp>
#include <string>

namespace config {

template <std::size_t N>
void ensureKeysExist(const nlohmann::json &jsonData,
                     const std::array<std::string, N> &keys) {
  for (const auto &key : keys) {
    if (!jsonData.contains(key)) {
      throw std::invalid_argument("Missing required key: " + key);
    }
  }
}

class ConfigReader {
public:
  ConfigReader(std::string configPath);

  // Need to make default constructor in cpp due to forward declare
  ~ConfigReader();

  acoustics::Grid2D readBathymetry() const;
  acoustics::Grid3D readSSP() const;

private:
  const std::string configPath_;
  nlohmann::json jsonData_;
};

} // namespace config
