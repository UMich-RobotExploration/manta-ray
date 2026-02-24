//
// Created by tko on 2/24/26.
//

#include "ConfigReader.h"

#include "Logger.h"

#include "fmt/format.h"
#include "fmt/ranges.h"
#include "spdlog/spdlog.h"
#include <filesystem>
#include <fstream>
#include <npy.hpp>

using json = nlohmann::json;

namespace config {

ConfigReader::ConfigReader(std::string configPath) : configPath_(configPath) {
  if (!std::filesystem::exists(configPath)) {
    auto msg = fmt::format("configPath does not exist at {}", configPath);
    throw std::invalid_argument(msg);
  }
  SPDLOG_INFO("Found config at: {}", configPath_);
  std::ifstream f;
  f.open(configPath_);
  jsonData_ = json::parse(f);
  SPDLOG_INFO("Parsed json config file.");
  SPDLOG_DEBUG("Resulting output is {}", jsonData_.dump());
  f.close();
}
ConfigReader::~ConfigReader() = default;

acoustics::Grid2D ConfigReader::readBathymetry() const {
  std::filesystem::path rootPath(jsonData_["source_dir"]);
  if (!std::filesystem::exists(rootPath)) {
    auto msg = fmt::format("source_dir from config does not exist {}",
                           rootPath.c_str());
    throw std::invalid_argument(msg);
  }

  auto &subJson = jsonData_["bathymetry"];
  ensureKeysExist<3>(subJson, {"data", "x", "y"});

  std::vector<double> bathymetry;
  std::vector<double> xCoords;
  std::vector<double> yCoords;
  for (auto &[key, value] : subJson.items()) {
    std::filesystem::path dataPath = rootPath / value;
    npy::npy_data d = npy::read_npy<double>(dataPath);
    if (d.fortran_order) {
      auto msg = fmt::format(
          "Do not save npy files in fortran order, force C-style order");
      throw std::invalid_argument(msg);
    }
    SPDLOG_TRACE("Data read in {}", d.data);
    SPDLOG_DEBUG("Key: {}. Shape of data {}", key, d.shape);
    if (key == "data") {
      bathymetry = d.data;
    } else if (key == "x") {
      xCoords = d.data;
    } else if (key == "y") {
      yCoords = d.data;
    } else {
      SPDLOG_WARN("Ignored bathymetry config key: {}, with values: {}", key,
                  value);
    }
  }
  return acoustics::Grid2D(xCoords, yCoords, bathymetry);
}

acoustics::Grid3D ConfigReader::readSSP() const {
  std::filesystem::path rootPath(jsonData_["source_dir"]);
  if (!std::filesystem::exists(rootPath)) {
    auto msg = fmt::format("source_dir from config does not exist {}",
                           rootPath.c_str());
    throw std::invalid_argument(msg);
  }

  auto &subJson = jsonData_["ssp"];
  ensureKeysExist<4>(subJson, {"data", "x", "y", "z"});

  std::vector<double> ssp;
  std::vector<double> xCoords;
  std::vector<double> yCoords;
  std::vector<double> zCoords;
  for (auto &[key, value] : subJson.items()) {
    std::filesystem::path dataPath = rootPath / value;
    npy::npy_data d = npy::read_npy<double>(dataPath);
    if (d.fortran_order) {
      auto msg = fmt::format(
          "Do not save npy files in fortran order, force C-style order");
      throw std::invalid_argument(msg);
    }
    SPDLOG_TRACE("Data read in {}", d.data);
    SPDLOG_DEBUG("Key: {}. Shape of data {}", key, d.shape);
    if (key == "data") {
      ssp = d.data;
    } else if (key == "x") {
      xCoords = d.data;
    } else if (key == "y") {
      yCoords = d.data;
    } else if (key == "z") {
      zCoords = d.data;
    } else {
      SPDLOG_WARN("Ignored bathymetry config key: {}, with values: {}", key,
                  value);
    }
  }
  return acoustics::Grid3D(xCoords, yCoords, zCoords, ssp);
}
} // namespace config