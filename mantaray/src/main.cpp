
#include <bhc/bhc.hpp>
#include <filesystem>
#include <random>
#include <vector>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

// #define BHC_DLL_IMPORT 1
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

std::ostream &operator<<(std::ostream &out, const bhc::rayPt<true> &x) {
  out << x.NumTopBnc << " " << x.NumBotBnc << " " << x.x.x << " " << x.x.y
      << " " << x.t.x << " " << x.t.y << " " << x.c << " " << x.Amp << " "
      << x.Phase << " " << x.tau;
  return out;
}

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
  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "A");
  // Important to set to I for irregular grid tracking
  context.params().Beam->RunType[4] = 'I';
  strcpy(context.params().Title, runName);

  //////////////////////////////////////////////////////////////////////////////
  // Bathymetry Setup
  //////////////////////////////////////////////////////////////////////////////

  std::vector<double> bathGridX =
      acoustics::utils::linspace<double>(-10, 55, 10);
  std::vector<double> bathGridY =
      acoustics::utils::linspace<double>(-11, 30, 9);
  std::vector<double> bathData;
  double bathDepth = 5.0;
  acoustics::AcousticsBuilder::quadraticBathymetry3D(bathGridX, bathGridY,
                                                     bathData, bathDepth);
  acoustics::BathymetryConfig bathConfig = acoustics::BathymetryConfig{
      acoustics::Grid2D(bathGridX, bathGridY, bathData),
      acoustics::BathyInterpolationType::kLinear, true};

  //////////////////////////////////////////////////////////////////////////////
  // SSP Setup
  //////////////////////////////////////////////////////////////////////////////
  int nX = 10;
  int nY = 10;
  int nZ = 100;
  const double refSoundSpeed = 1500.0;
  auto SSPgridX = acoustics::utils::linspace(-10.0, 55.0, nX);
  auto SPPgridY = acoustics::utils::linspace(-30.0, 30.0, nY);
  auto SSPgridZ = acoustics::utils::linspace(0.0, 5000.0 / 1000.0, nZ);
  auto SSPGrid = acoustics::Grid3D(SSPgridX, SPPgridY, SSPgridZ, refSoundSpeed);
  // acoustics::munkProfile(SSPGrid, refSoundSpeed, true);

  auto sspConfig = acoustics::SSPConfig{std::move(SSPGrid), true};

  //////////////////////////////////////////////////////////////////////////////
  // Source / Receivers Setup
  //////////////////////////////////////////////////////////////////////////////
  Eigen::Vector3d sourcePos(10.0, 0.0, 1000.0);
  Eigen::Vector3d receiverPos;
  receiverPos(0) = 50000.0;
  receiverPos(1) = 10.0;
  receiverPos(2) = 1000.0;

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
  world.addLandmark(Eigen::Vector3d(-10001.0, 100.0, 10.0));
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

  bhc::writeout(context.params(), context.outputs(), runName);
  return 0;
}
