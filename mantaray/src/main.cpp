
#include <bhc/bhc.hpp>
#include <filesystem>
#include <random>
#include <vector>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

// #define BHC_DLL_IMPORT 1
#include "ConfigReader.h"
#include "acoustics/Arrival.h"
#include "acoustics/BellhopContext.h"
#include "acoustics/acousticsConstants.h"
#include "acoustics/helpers.h"

#include "acoustics/AcousticsBuilder.h"
#include "acoustics/Grid.h"
#include "acoustics/SimulationConfig.h"
#include "rb/RbWorld.h"
#include "rb/RobotsAndSensors.h"

#include <Logger.h>

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
  rb::RbWorld world{};
  double endTime = 50.0;
  world.simData.dt = 0.1;
  world.createRngEngine(10020);
  world.reserveRobots(1);
  world.reserveLandmarks(2);
  auto odomRobotIdx =
      world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(0.1, 0.0, 1.0));
  auto gtIdx = world.robots[odomRobotIdx]->addSensor(
      std::make_unique<rb::GroundTruthPose>(
          10.0, static_cast<size_t>(100.0 * endTime)));
  auto odomIdx = world.robots[odomRobotIdx]->addSensor(
      std::make_unique<rb::PositionalXYOdomoetry>(
          10.0, static_cast<size_t>(100.0 * endTime),
          std::normal_distribution<double>{0.0, 0.001}));
  auto robotIdx2 =
      world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(0.0, 0.0, 5.0));
  world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(1.0, 0.0, 5.0));
  world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(1.0, 4.0, 5.0));
  auto &kinData = world.dynamicsBodies.getKinematicData(odomRobotIdx);
  auto &pose = kinData.poseGlobal.coeffs();
  world.addLandmark(Eigen::Vector3d(-100.0, 100.0, 10.0));
  auto result = simBuilder.updateSource(world.landmarks[0]);
  if (result != acoustics::BoundaryCheck::kInBounds) {
    SPDLOG_ERROR("Landmark is not valid and we can't run the sim.");
    return 0;
  }

  auto startTime = world.simData.time;
  while (startTime < endTime) {
    startTime += 1.0 + 1.0 / 3.0;
    world.advanceWorld(startTime);
    if (std::remainder(startTime, 2.0) < 1e-6) {
      for (auto &robot : world.robots) {
        auto position = world.dynamicsBodies.getPosition(robot->getBodyIdx());
        auto result = simBuilder.updateReceiver(position);
        if (result != acoustics::BoundaryCheck::kInBounds) {
          SPDLOG_INFO("Robot ID: {} is being marked as dead",
                      robot->getBodyIdx());
          robot->isAlive_ = false;
          continue;
        }

        bellhop_logger->debug("\n===Start Bellhop Run===\n");

        if (bellhop_logger->level() == spdlog::level::debug) {
          bhc::echo(context.params());
        }
        bhc::run(context.params(), context.outputs());
        bellhop_logger->debug("\n===End Bellhop Run===\n");

        auto arrival = acoustics::Arrival(context.params(), context.outputs());
        auto earliestArrival = arrival.getFastestArrival();
        SPDLOG_INFO("Received arrival time: {}", earliestArrival);
        if (earliestArrival < 0) {
          SPDLOG_WARN("Received no arrival time");
        }
        float currSsp = 0;
        auto pos = acoustics::utils::safeEigenToVec23(position);

        bhc::get_ssp<true, true>(context.params(), pos, currSsp);
      }
    }
  }
  rb::outputRobotSensorToCsv("simTest", *world.robots[odomRobotIdx]);

  bhc::writeenv(context.params(), runName);
  bhc::writeout(context.params(), context.outputs(), runName);
  return 0;
}
