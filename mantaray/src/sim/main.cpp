
#include <bhc/bhc.hpp>
#include <filesystem>
#include <random>

// Logger.h must be included before spdlog/spdlog.h to define macros
#include <mantaray/utils/Logger.h>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

// #define BHC_DLL_IMPORT 1
#include "acoustics/Arrival.h"
#include "acoustics/BellhopContext.h"
#include "acoustics/acousticsConstants.h"
#include "acoustics/helpers.h"
#include <mantaray/config/EnvironmentConfig.h>
#include <mantaray/config/SimConfig.h>

#include "acoustics/AcousticsBuilder.h"
#include "acoustics/Grid.h"
#include "acoustics/SimulationConfig.h"
#include "rb/RbWorld.h"
#include "rb/RobotsAndSensors.h"
#include <mantaray/sim/AcousticPairwiseRangeSystem.h>
#include <mantaray/sim/CurrentDriftRobot.h>
#include <mantaray/sim/RobotFactory.h>
#include <mantaray/utils/PfgWriter.h>

void PrtCallback(const char *message) { bellhop_logger->debug("{}", message); }
void OutputCallback(const char *message) {
  bellhop_logger->debug("{}", message);
}

config::SimConfig parseArgs(int argc, char *argv[]) {
  if (argc < 2) {
    fmt::print(stderr, "Usage: {} <sim_config.json>\n", argv[0]);
    std::exit(1);
  }

  auto config = config::loadSimConfig(argv[1]);

  auto outDir = std::filesystem::path(config.outputDir);
  if (std::filesystem::exists(outDir) && !std::filesystem::is_empty(outDir)) {
    fmt::print(stderr, "Error: output directory is not empty: {}\n",
               config.outputDir);
    std::exit(1);
  }
  std::filesystem::create_directories(outDir);

  return config;
}

int main(int argc, char *argv[]) {

  auto config = parseArgs(argc, argv);
  auto outDir = std::filesystem::path(config.outputDir);

  init_logger(config.outputDir);
  spdlog::set_default_logger(global_logger);

  SPDLOG_INFO("Loaded sim config: {}", argv[1]);
  SPDLOG_INFO("Beginning Bellhop Robotics Sim");
  SPDLOG_INFO("Run name: {}", config.runName);
  SPDLOG_INFO("Output directory: {}", config.outputDir);

  auto init = bhc::bhcInit();
  init.FileRoot = nullptr;
  init.prtCallback = PrtCallback;
  init.outputCallback = OutputCallback;
  init.maxMemory = config.bellhopMemoryMib * 1024ull * 1024ull;
  init.numThreads = -1;

  auto envConfig = config::EnvironmentConfig(config.envConfigFile);
  auto importedBathGrid = envConfig.readBathymetry();
  auto importedSSPGrid = envConfig.readSSP();
  auto importedCurrentGrid = envConfig.readCurrent();

  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "A");
  // Important to set to I for irregular grid tracking
  context.params().Beam->RunType[4] = 'I';
  strncpy(context.params().Title, config.runName.c_str(),
          sizeof(context.params().Title) - 1);

  auto bathConfig = acoustics::BathymetryConfig{
      std::move(importedBathGrid), acoustics::BathyInterpolationType::kLinear,
      false};

  auto sspConfig = acoustics::SSPConfig{std::move(importedSSPGrid), false};

  acoustics::AgentsConfig agents{{0, 0, 10}, {0, 0, 1}};
  auto simBuilder = acoustics::AcousticsBuilder(context.params(), bathConfig,
                                                sspConfig, agents);
  simBuilder.build();

  // Simulation setup
  double endTime = config.endTimeHours * 3600.0;
  rb::RbWorld world{};
  world.simData.dt = config.physicsDt;
  world.createRngEngine(config.rngSeed);

  // Create robots from config
  std::vector<rb::RobotIdx> robotIndices;
  for (const auto &rj : config.robotsJson) {
    auto type = config::robotTypeFromString(rj.at("type"));
    rb::RobotIdx idx;
    switch (type) {
    case config::RobotType::kConstantVel: {
      auto cfg = rj.get<rb::ConstantVelConfig>();
      idx = sim::addStandardRobot<rb::ConstantVelRobot>(
          world, endTime, cfg.position, config.sensors, cfg);
      break;
    }
    case config::RobotType::kCurrentDrift: {
      auto cfg = rj.get<robots::CurrentDriftConfig>();
      idx = sim::addStandardRobot<robots::CurrentDriftRobot>(
          world, endTime, cfg.position, config.sensors, importedCurrentGrid,
          cfg);
      break;
    }
    }
    robotIndices.push_back(idx);
  }

  for (const auto &lm : config.landmarks) {
    world.addLandmark(lm);
  }

  // TOF mode
  sim::GlobalTofMode tofMode = (config.tofMode == "two_way")
                                   ? sim::GlobalTofMode::kTwoWay
                                   : sim::GlobalTofMode::kOneWay;

  sim::AcousticPairwiseRangeSystem rangeSystem(simBuilder, context, tofMode);
  rangeSystem.rebuildPairs(world);

  double boundsCheckInterval = config.boundsCheckIntervalSec;
  double pingInterval = config.pingIntervalMin * 60.0;
  double nextBoundsCheck = world.simData.time + boundsCheckInterval;
  double nextPing = world.simData.time + pingInterval;

  auto startTime = world.simData.time;
  while (startTime < endTime) {
    double nextEvent = std::min(nextBoundsCheck, nextPing);
    startTime = nextEvent;
    world.advanceWorld(startTime);

    if (startTime >= nextBoundsCheck) {
      rangeSystem.checkBounds(world);
      nextBoundsCheck += boundsCheckInterval;
    }
    if (startTime >= nextPing) {
      rangeSystem.update(startTime, world);
      nextPing += pingInterval;
    }
  }
  SPDLOG_INFO("Pairwise acoustic links: {}, measurements logged: {}",
              rangeSystem.getLinks().size(),
              rangeSystem.getMeasurements().size());

  // Write sensor CSVs
  for (size_t i = 0; i < robotIndices.size(); ++i) {
    auto csvBase = (outDir / fmt::format("robot_{}", i)).string();
    rb::outputRobotSensorToCsv(csvBase.c_str(), *world.robots[robotIndices[i]]);
  }

  // Write PFG factor graph file
  auto pfgPath = (outDir / "output.pfg").string();
  pyfg::writePfg(pfgPath, world, rangeSystem.getMeasurements(), config.pfg);

  auto bellhopBase = (outDir / config.runName).string();
  bhc::writeenv(context.params(), bellhopBase.c_str());
  bhc::writeout(context.params(), context.outputs(), bellhopBase.c_str());
  return 0;
}
