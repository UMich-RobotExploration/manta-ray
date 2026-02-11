
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>
#include <bhc/bhc.hpp>
#include <filesystem>

// #define BHC_DLL_IMPORT 1
#include "acoustics/Arrival.h"
#include "acoustics/helpers.h"
#include "acoustics/acousticsConstants.h"
#include "acoustics/BhHandler.h"

#include "acoustics/AcousticsBuilder.h"
#include "acoustics/Grid.h"
#include "acoustics/SimulationConfig.h"
#include "rb/PhysicsBodies.h"
#include "rb/include/rb/PhysicsBodies.h"

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
  // init.maxMemory = 80ull * 1024ull * 1024ull; // 30 MiB
  init.numThreads = -1;
  init.useRayCopyMode = true;
  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "R");
  // Important to set to I for irregular grid tracking
  context.params().Beam->RunType[4] = 'I';
  strcpy(context.params().Title, runName);

  //////////////////////////////////////////////////////////////////////////////
  // Bathymetry Setup
  //////////////////////////////////////////////////////////////////////////////

  std::vector<double> bathGridX =
      acoustics::utils::linspace<double>(-1, 55, 10);
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
  auto SSPgridX = acoustics::utils::linspace(-1.0, 55.0, nX);
  auto SPPgridY = acoustics::utils::linspace(-30.0, 30.0, nY);
  auto SSPgridZ = acoustics::utils::linspace(0.0, 5000.0 / 1000.0, nZ);
  auto SSPGrid =
      acoustics::Grid3D(SSPgridX, SPPgridY, SSPgridZ, 1500.0);
  acoustics::munkProfile(SSPGrid, 1500.0, true);

  auto sspConfig =
      acoustics::SSPConfig{std::move(SSPGrid), true};
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

  for (size_t i = 0; i < acoustics::kNumRecievers; ++i) {
    float sspVal = 0;
    bhc::VEC23<true> vec = {receiverPos(0), receiverPos(1), receiverPos(2)};
    bhc::get_ssp<true, true>(context.params(), vec, sspVal);
    std::cout << "Receiver " << i << ": " << context.params().Pos->Rr[i] << ", "
              << context.params().Pos->theta[i] << " ,"
              << "Estimate TOF: " << (receiverPos - sourcePos).norm() / sspVal
              << "\n";
  }
  const acoustics::SSPConfig &sspConfigBuilt = simBuilder.getSSPConfig();
  for (size_t iz = 0; iz < sspConfigBuilt.Grid.nz(); ++iz) {
    float sspVal = 0;
    bhc::VEC23<true> vec = {0.0, 0.0,
                            sspConfigBuilt.Grid.zCoords.at(iz) * 1000.0};
    bhc::get_ssp<true, true>(context.params(), vec, sspVal);
    std::cout << "Z height (meters) "
              << sspConfigBuilt.Grid.zCoords.at(iz) * 1000.0
              << ", Bellhop vs Build in Data structure: (" << sspVal << ", "
              << sspConfigBuilt.Grid.at(0, 0, iz) << ")"
              << "\n";
  }

  std::cout << "Run type: " << context.params().Beam->RunType << "\n";
  std::cout << "Box x: " << context.params().Beam->Box.x << "\n";
  std::cout << "Box y: " << context.params().Beam->Box.y << "\n";
  std::cout << "Box z: " << context.params().Beam->Box.z << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->bot.NPts.x << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->top.NPts.y << "\n";
  try {
    bhc::echo(context.params());
  } catch (const std::exception &e) {
    std::cerr << "Error during echo: " << e.what() << std::endl;
    return 1;
  }
  bhc::writeenv(context.params(), runName);
  for (int i = 0; i < 1; ++i) {
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();

    bhc::run(context.params(), context.outputs());

    std::chrono::nanoseconds delta =
        std::chrono::high_resolution_clock::now() - t1;

    auto &agentConfig = simBuilder.getAgentsConfig();
    std::cout << "Receiver Location: " << agentConfig.receiver << "\n";
    simBuilder.updateReceiver(agentConfig.receiver(0) += 100,
                              agentConfig.receiver(1) += 100,
                              agentConfig.receiver(2) += 25);

    try {
      auto arrival = acoustics::Arrival(context.params(), context.outputs());
      auto arrivalVec = arrival.extractEarliestArrivals();
      acoustics::utils::printVector(arrivalVec);
    } catch (const std::exception &e) {
      std::cerr << "Error during arrival extraction: " << e.what() << std::endl;
    }
    std ::cout
        << "Iteration " << i << " took "
        << std::chrono::duration_cast<std::chrono::milliseconds>(delta).count()
        << " ms\n";
  }
  bhc::writeout(context.params(), context.outputs(), runName);
  return 0;
}
