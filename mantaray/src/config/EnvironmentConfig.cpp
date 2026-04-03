//
// Created by tko on 2/24/26.
//

#include <mantaray/config/EnvironmentConfig.h>

#include "mantaray/utils/Logger.h"

#include "fmt/format.h"
#include "fmt/ranges.h"
#include "spdlog/spdlog.h"
#include <filesystem>
#include <fstream>
#include <npy.hpp>

using json = nlohmann::json;

namespace config {

EnvironmentConfig::EnvironmentConfig(std::string configPath)
    : configPath_(configPath) {
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
EnvironmentConfig::~EnvironmentConfig() = default;

acoustics::Grid2D EnvironmentConfig::readBathymetry() const {
  const std::string subKey = "bathymetry";
  auto rootPath = validateJSON<3>(jsonData_, subKey, {"data", "x", "y"});
  auto &subJson = jsonData_[subKey];

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

acoustics::Grid3D EnvironmentConfig::readSSP() const {
  const std::string subKey = "ssp";
  auto rootPath = validateJSON<4>(jsonData_, subKey, {"data", "x", "y", "z"});
  auto &subJson = jsonData_[subKey];

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
acoustics::GridVec EnvironmentConfig::readCurrent() const {
  const std::string subKey = "current";
  auto rootPath = validateJSON<4>(jsonData_, subKey, {"x", "y", "u", "v"});
  auto &subJson = jsonData_[subKey];

  std::vector<double> xCoords;
  std::vector<double> yCoords;
  std::vector<double> zCoords;
  std::vector<double> u;
  std::vector<double> v;
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
    if (key == "x") {
      xCoords = d.data;
    } else if (key == "y") {
      yCoords = d.data;
    } else if (key == "z") {
      zCoords = d.data;
    } else if (key == "u") {
      u = d.data;
    } else if (key == "v") {
      v = d.data;
    } else {
      SPDLOG_WARN("Ignored bathymetry config key: {}, with values: {}", key,
                  value);
    }
  }

  const size_t expected = xCoords.size() * yCoords.size() * zCoords.size();

  std::vector<Eigen::Vector2d> field;
  field.reserve(expected);
  if (u.size() != expected || v.size() != expected) {
    auto errMsg = fmt::format("U size: {}, V size: {}, but expects: {}",
                              u.size(), v.size(), expected);
    throw std::invalid_argument(errMsg);
  }
  for (size_t i = 0; i < expected; ++i) {
    field.emplace_back(u[i], v[i]);
  }

  return acoustics::GridVec(std::move(xCoords), std::move(yCoords),
                            std::move(zCoords), std::move(field));
}
} // namespace config