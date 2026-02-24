//
// Created by tko on 2/24/26.
//

#pragma once
#include <acoustics/Grid.h>
#include <json.hpp>
#include <string>

namespace config {

/**
 * @brief ConfigReader is responsible for reading and parsing configuration
 * files.
 *
 * This class provides methods to load and validate configuration data from JSON
 * files. It supports reading bathymetry and sound speed profile (SSP) data,
 * ensuring that required keys are present and the data is properly formatted.
 */
class ConfigReader {
public:
  /**
   * @brief Constructs a ConfigReader with the specified configuration file
   * path.
   *
   * This constructor validates the existence of the configuration file and
   * parses its contents into a JSON object.
   *
   * @param configPath Path to the configuration file.
   * @throws std::invalid_argument if the file does not exist or cannot be
   * parsed.
   */
  ConfigReader(std::string configPath);

  /**
   * @brief Default destructor for ConfigReader.
   */
  ~ConfigReader();

  /**
   * @brief Reads and constructs a 2D bathymetry grid from the configuration
   * file.
   *
   * This method extracts bathymetry data, x-coordinates, and y-coordinates from
   * the configuration file and constructs a 2D grid. It ensures that the
   * required keys ("data", "x", "y") are present in the JSON data.
   *
   * Data validation is conduced by acoustics::Grid2D and should not be handled
   * by this method.
   *
   * @return A 2D grid representing the bathymetry data.
   * @throws std::invalid_argument if required keys are missing or data is
   * invalid.
   */
  acoustics::Grid2D readBathymetry() const;

  /**
   * @brief Reads and constructs a 3D sound speed profile (SSP) grid from the
   * configuration file.
   *
   * This method extracts SSP data, x-coordinates, y-coordinates, and
   * z-coordinates from the configuration file and constructs a 3D grid. It
   * ensures that the required keys ("data", "x", "y", "z") are present in the
   * JSON data.
   *
   *
   * Data validation is conduced by acoustics::Grid3D and should not be handled
   * by this method.
   *
   * @return A 3D grid representing the sound speed profile data.
   * @throws std::invalid_argument if required keys are missing or data is
   * invalid.
   */
  acoustics::Grid3D readSSP() const;

private:
  // Path to the configuration file.
  const std::string configPath_;
  // Parsed JSON data from the configuration file.
  nlohmann::json jsonData_;
};

/**
 * @brief Checks that json or json sub dictionary has all required keys
 */
template <std::size_t N>
void ensureKeysExist(const nlohmann::json &jsonData,
                     const std::array<std::string, N> &keys) {
  for (const auto &key : keys) {
    if (!jsonData.contains(key)) {
      throw std::invalid_argument("Missing required key: " + key);
    }
  }
}

} // namespace config
