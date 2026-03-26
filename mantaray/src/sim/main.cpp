
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
#include <mantaray/config/ConfigReader.h>

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

int main() {

  init_logger();
  auto init = bhc::bhcInit();
  // Set the global logger as the default logger
  spdlog::set_default_logger(global_logger);
  SPDLOG_INFO("Beginning Bellhop Robotics Sim");

  char runName[] = "overhaul";
  SPDLOG_INFO("Run name: {}", runName);
  SPDLOG_INFO("Current run path is: {}",
              std::filesystem::current_path().c_str());
  init.FileRoot = nullptr;
  init.prtCallback = PrtCallback;
  init.outputCallback = OutputCallback;
  // Profiled memory to find PreProcess was the longest task in the sim
  // Reducing memory is the only way to limit it's overhead.
  init.maxMemory = 80ull * 1024ull * 1024ull; // 80 MiB
  init.numThreads = -1;
  // init.useRayCopyMode = true;

  auto configFile = config::ConfigReader("../../sim_config/monterey.json");
  auto importedBathGrid = configFile.readBathymetry();
  auto importedSSPGrid = configFile.readSSP();
  auto importedCurrentGrid = configFile.readCurrent();

  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "A");
  // Important to set to I for irregular grid tracking
  context.params().Beam->RunType[4] = 'I';
  strcpy(context.params().Title, runName);

  //////////////////////////////////////////////////////////////////////////////
  // Bathymetry Setup
  //////////////////////////////////////////////////////////////////////////////

  acoustics::BathymetryConfig bathConfig = acoustics::BathymetryConfig{
      std::move(importedBathGrid), acoustics::BathyInterpolationType::kLinear,
      false};

  //////////////////////////////////////////////////////////////////////////////
  // SSP Setup
  //////////////////////////////////////////////////////////////////////////////

  auto sspConfig = acoustics::SSPConfig{std::move(importedSSPGrid), false};

  //////////////////////////////////////////////////////////////////////////////
  // Source / Receivers Setup
  //////////////////////////////////////////////////////////////////////////////
  Eigen::Vector3d sourcePos(0.0, 0.0, 1000.0);
  Eigen::Vector3d receiverPos;
  receiverPos(0) = 0.0;
  receiverPos(1) = 0.0;
  receiverPos(2) = 1.0;

  acoustics::AgentsConfig agents =
      acoustics::AgentsConfig{sourcePos, receiverPos};

  acoustics::AcousticsBuilder simBuilder = acoustics::AcousticsBuilder(
      context.params(), bathConfig, sspConfig, agents);
  simBuilder.build();

  //////////////////////////////////////////////////////////////////////////////
  // Simulation Setup
  //////////////////////////////////////////////////////////////////////////////

  /// How long do we want to run?
  /// Multiple days
  /// Every few hours on ranging?
  /// Odometry I'll do hours
  /// Need to figure out DT

  rb::RbWorld world{};
  double endTime = 60.0 * 60.0;
  world.simData.dt = 0.1;
  world.createRngEngine(10020);
  world.reserveRobots(4);
  world.reserveLandmarks(1);

  sim::StandardSensorConfig sensorCfg{};

  sim::addStandardRobot<rb::ConstantVelRobot>(
      world, endTime, Eigen::Vector3d(100.0, 0.0, 0.01), sensorCfg,
      Eigen::Vector3d(0.1, 0.3, 1.0));

  auto robotIdx2 = sim::addStandardRobot<robots::CurrentDriftRobot>(
      world, endTime, Eigen::Vector3d(100.0, -10000.0, 0.01), sensorCfg,
      importedCurrentGrid, 2000.0);

  sim::addStandardRobot<rb::ConstantVelRobot>(
      world, endTime, Eigen::Vector3d(100.0, 1000.0, 0.01), sensorCfg,
      Eigen::Vector3d(1.0, 0.0, 5.0));

  sim::addStandardRobot<rb::ConstantVelRobot>(
      world, endTime, Eigen::Vector3d(-100.0, -1000.0, 0.01), sensorCfg,
      Eigen::Vector3d(1.0, 4.0, 5.0));

  world.addLandmark(Eigen::Vector3d(-10001.0, 100.0, 10.0));

  sim::AcousticPairwiseRangeSystem rangeSystem(simBuilder, context,
                                               sim::GlobalTofMode::kOneWay);
  rangeSystem.rebuildPairs(world);

  constexpr double kBoundsCheckInterval = 10.0;
  constexpr double kPingInterval = 60.0;
  double nextBoundsCheck = world.simData.time + kBoundsCheckInterval;
  double nextPing = world.simData.time + kPingInterval;

  auto startTime = world.simData.time;
  while (startTime < endTime) {
    double nextEvent = std::min(nextBoundsCheck, nextPing);
    startTime = nextEvent;
    world.advanceWorld(startTime);

    if (startTime >= nextBoundsCheck) {
      rangeSystem.checkBounds(world);
      nextBoundsCheck += kBoundsCheckInterval;
    }
    if (startTime >= nextPing) {
      rangeSystem.update(startTime, world);
      nextPing += kPingInterval;
    }
  }
  SPDLOG_INFO("Pairwise acoustic links: {}, measurements logged: {}",
              rangeSystem.getLinks().size(),
              rangeSystem.getMeasurements().size());
  rb::outputRobotSensorToCsv("simTest", *world.robots[robotIdx2]);

  // Write PFG factor graph file
  pyfg::PfgWriterConfig pfgConfig{};
  pfgConfig.useGroundTruthOdometry = true;
  pfgConfig.rangeVariance = 1.0;
  pfgConfig.defaultPosePriorCov =
      pyfg::makeDiagUpperTri6x6(0.01, 0.01, 0.01, 0.04, 0.04, 0.04);
  pfgConfig.landmarkPriorCovs.push_back(
      pyfg::makeDiagUpperTri3x3(0.01, 0.01, 0.01));
  pfgConfig.odomRotationVariance = 0.04;
  pyfg::writePfg("output.pfg", world, rangeSystem.getMeasurements(), pfgConfig);

  bhc::writeenv(context.params(), runName);
  bhc::writeout(context.params(), context.outputs(), runName);
  return 0;
}
