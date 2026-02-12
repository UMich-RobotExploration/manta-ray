
#include <atomic>
#include <bhc/bhc.hpp>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <vector>

// #define BHC_DLL_IMPORT 1
#include "acoustics/Arrival.h"
#include "acoustics/BhHandler.h"
#include "acoustics/acousticsConstants.h"
#include "acoustics/helpers.h"

#include "acoustics/AcousticsBuilder.h"
#include "acoustics/Grid.h"
#include "acoustics/SimulationConfig.h"
#include "rb/ConstantVelRobot.h"
#include "rb/RbWorld.h"

std::ostream &operator<<(std::ostream &out, const bhc::rayPt<true> &x) {
  out << x.NumTopBnc << " " << x.NumBotBnc << " " << x.x.x << " " << x.x.y
      << " " << x.t.x << " " << x.t.y << " " << x.c << " " << x.Amp << " "
      << x.Phase << " " << x.tau;
  return out;
}

void PrtCallback(const char *message) { std::cout << message << std::flush; }
void OutputCallback(const char *message) {
  std::cout << "Out: " << message << std::endl << std::flush;
}

int main() {

  auto init = bhc::bhcInit();

  char runName[] = "overhaul";
  std::cout << "Current path is " << std::filesystem::current_path()
            << std::endl;
  init.FileRoot = nullptr;
  init.prtCallback = PrtCallback;
  init.outputCallback = OutputCallback;
  // Profiled memory to find PreProcess was the longest task in the sim
  // Reducing memory is the only way to limit it's overhead.
  init.maxMemory = 80ull * 1024ull * 1024ull; // 30 MiB
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
  auto SSPgridX = acoustics::utils::linspace(-10.0, 55.0, nX);
  auto SPPgridY = acoustics::utils::linspace(-30.0, 30.0, nY);
  auto SSPgridZ = acoustics::utils::linspace(0.0, 5000.0 / 1000.0, nZ);
  auto SSPGrid = acoustics::Grid3D(SSPgridX, SPPgridY, SSPgridZ, 1500.0);
  acoustics::munkProfile(SSPGrid, 1500.0, true);

  auto sspConfig = acoustics::SSPConfig{std::move(SSPGrid), true};
  std::cout << "SSP at (0,0,z): " << std::endl;

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
  world.reserveRobots(1);
  world.reserveLandmarks(2);
  auto robotIdx =
      world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(0.1, 0.0, 1.0));
  auto robotIdx2 =
      world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(0.0, 0.0, 5.0));
  world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(1.0, 0.0, 5.0));
  world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(1.0, 4.0, 5.0));
  world.addLandmark(Eigen::Vector3d(10.0, 100.0, 10.0));
  simBuilder.updateSource(world.landmarks[0](0), world.landmarks[0](1),
                          world.landmarks[0](2));
  auto startTime = 0.0;
  while (startTime < 100.0) {
    startTime += 1.0;
    world.advanceWorld(startTime);
    if (std::remainder(startTime, 10.0) < 1e-6) {
      for (auto &robot : world.robots) {
        auto &kinematicData =
            world.dynamicsBodies.getKinematicData(robot->getBodyIdx());
        simBuilder.updateReceiver(kinematicData.poseGlobal.translation()(0),
                                  kinematicData.poseGlobal.translation()(1),
                                  kinematicData.poseGlobal.translation()(2));
        std::stringstream msg;
        msg << "\n/////////////////////////////////\n" ;
        msg << "Robot (" + std::to_string(robot->bodyIdx_) + ") at time " + std::to_string(startTime);
        msg << ", position [meters]: "
            << kinematicData.poseGlobal.translation().transpose() << "\n";
        std::cout << msg.str();
        msg.str("");
        bhc::run(context.params(), context.outputs());
        auto arrival = acoustics::Arrival(context.params(), context.outputs());
        auto arrivalVec = arrival.extractEarliestArrivals();
        acoustics::utils::printVector(arrivalVec);
      }
    }
  }

  try {
    bhc::echo(context.params());
  } catch (const std::exception &e) {
    std::cerr << "Error during echo: " << e.what() << std::endl;
    return 1;
  }
  bhc::writeout(context.params(), context.outputs(), runName);
  return 0;
}
