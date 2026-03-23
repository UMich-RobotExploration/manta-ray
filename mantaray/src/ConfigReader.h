/** @file ConfigReader.h
 * @brief Configuration reading and validation
 */

#pragma once
#include <acoustics/Grid.h>
#include <json.hpp>
#include <string>

namespace config {

// Defines key to look for source directory path of data
constexpr char kSourceKey[] = "source_dir";

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

  /**
   * @brief Reads and constructs a 3D current grid from the
   * configuration file.
   *
   * This method extracts current data as separate u/v components on an (x,y,z)
   * grid and constructs a GridVec grid.
   *
   * Expected JSON keys under "current":
   * - "x": path to x-coordinates npy
   * - "y": path to y-coordinates npy
   * - "z": path to z-coordinates npy
   * - "u": path to u-component npy (flattened to match Grid2D row-major)
   * - "v": path to v-component npy (flattened to match Grid2D row-major)
   *
   * @note GridVec is (x,y,z)
   *
   *
   * Data validation is conduced by acoustics::GridVec and should not be handled
   * by this method.
   *
   * @return A GridVec grid representing the sound speed profile data.
   * @throws std::invalid_argument if required keys are missing or data is
   * invalid.
   */
  acoustics::GridVec readCurrent() const;

private:
  // Path to the configuration file.
  const std::string configPath_;
  // Parsed JSON data from the configuration file.
  nlohmann::json jsonData_;
};

/**
 * @brief Validates that json structure has proper keys and subkeys
 *
 * @tparam N number of subkey to check for
 *
 * @details Ensures the following structure exists
 *
 * ```json
 *  {
 *    "source_dir" : "some_path",
 *      "subKey": {
 *        "key1": 1,
 *        "key2": 2
 *      }
 *  }
 * ```
 */
template <std::size_t N>
std::filesystem::path validateJSON(const nlohmann::json &jsonData,
                                   const std::string subKey,
                                   const std::array<std::string, N> &keys) {
  if (!jsonData.contains(kSourceKey)) {
    auto err = fmt::format("Missing required key: ", kSourceKey);
    throw std::invalid_argument(err);
  }
  if (!jsonData.contains(subKey)) {
    auto err = fmt::format("Missing required sub-key: ", subKey);
    throw std::invalid_argument(err);
  }
  std::filesystem::path rootPath(jsonData[kSourceKey]);
  if (!std::filesystem::exists(rootPath)) {
    auto msg = fmt::format("source_dir from config does not exist {}",
                           rootPath.c_str());
    throw std::invalid_argument(msg);
  }
  for (const auto &key : keys) {
    if (!jsonData[subKey].contains(key)) {
      throw std::invalid_argument("Missing required key: " + key);
    }
  }
  return rootPath;
}

} // namespace config
