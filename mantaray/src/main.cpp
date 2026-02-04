
#include <atomic>
#include <chrono>
#include <cstring>
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
#include <bhc/bhc.hpp>
#include <filesystem>

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
  // init.FileRoot = "manual";
  init.FileRoot = nullptr;
  init.prtCallback = PrtCallback;
  init.outputCallback = OutputCallback;
  // Profiled memory to find PreProcess was the longest task in the sim
  // Reducing memory is the only way to limit it's overhead.
  init.maxMemory = 30ull * 1024ull * 1024ull; // 30 MiB
  init.numThreads = static_cast<int32_t>(1);
  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "A");
  // Important to set to I for irregular grid tracking
  context.params().Beam->RunType[4] = 'I';
  strcpy(context.params().Title, runName);

  //////////////////////////////////////////////////////////////////////////////
  // Bathymetry Setup
  //////////////////////////////////////////////////////////////////////////////

  std::vector<double> bathGridX =
      acoustics::utils::linspace<double>(-10, 10, 10);
  std::vector<double> bathGridY =
      acoustics::utils::linspace<double>(-11, 11, 10);
  std::vector<double> bathData;
  double bathDepth = 5.0;
  acoustics::AcousticsBuilder::quadraticBathymetry3D(bathGridX, bathGridY,
                                                     bathData, bathDepth);
  // acoustics::utils::printVector(bathGridX);
  // acoustics::utils::printVector(bathGridY);
  // acoustics::utils::printVector(bathData);
  acoustics::BathymetryConfig bathConfig = acoustics::BathymetryConfig{
      acoustics::Grid2D<double>(bathGridX, bathGridY, bathData),
      acoustics::BathyInterpolationType::kLinear, true};

  //////////////////////////////////////////////////////////////////////////////
  // SSP Setup
  //////////////////////////////////////////////////////////////////////////////
  int nX = 10;
  int nY = 10;
  int nZ = 10;
  auto SSPgridX = acoustics::utils::linspace(-10.0, 10.0, nX);
  auto SPPgridY = acoustics::utils::linspace(-10.0, 10.0, nY);
  auto SSPgridZ = acoustics::utils::linspace(0.0, 2000.0 / 1000.0, nZ);
  auto SSPdata = std::vector<double>(nX * nY * nZ, 1500.0);
  acoustics::SSPConfig sspConfig = acoustics::SSPConfig{
      acoustics::Grid3D<double>(SSPgridX, SPPgridY, SSPgridZ, 1500.0), true};
  // std::cout << sspConfig.Grid.at(0, 0, 0) << "\n";

  //////////////////////////////////////////////////////////////////////////////
  // Source / Receivers Setup
  //////////////////////////////////////////////////////////////////////////////
  Eigen::Vector3d sourcePos(10.0, 0.0, 100.0);
  Eigen::Vector3d receiverPos;
  receiverPos(0) = 0.0;
  receiverPos(1) = 50;
  receiverPos(2) = 100;

  acoustics::AgentsConfig agents =
      acoustics::AgentsConfig{sourcePos, receiverPos, false};

  acoustics::AcousticsBuilder simBuilder = acoustics::AcousticsBuilder(
      context.params(), bathConfig, sspConfig, agents);
  simBuilder.build();

  for (size_t i = 0; i < acoustics::kNumRecievers; ++i) {
    std::cout << "Receiver " << i << ": " << context.params().Pos->Rr[i] << ", "
              << context.params().Pos->theta[i] << " ,"
              << "Estimate TOF: " << (receiverPos - sourcePos).norm() / 1500.0
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
  for (int i = 0; i < 5; ++i) {
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();

    bhc::run(context.params(), context.outputs());

    std::chrono::nanoseconds delta =
        std::chrono::high_resolution_clock::now() - t1;

    auto &agentConfig = simBuilder.getAgentsConfig();
    // TODO: ONLY HAVE A SINGLE RECEIVER NOW DUMMY
    // just double^3 on these not Eigen
    // simBuilder.moveSource(x,y,z);
    // -> updateAgents
    // simBuilder.moveReciever(x,y,z);
    // -> updateAgents
    // Need to consider a 2D arrival time matrix where (i,j) relates agents arrival times
    agentConfig.receiver(0) += 100;
    agentConfig.receiver(1) += 100;
    agentConfig.receiver(2) += 25;
    std::cout << "Receiver Location: " << agentConfig.receiver << "\n";
    simBuilder.updateAgents();

    auto arrival = acoustics::Arrival(context.params(), context.outputs());
    auto arrivalVec = arrival.extractEarliestArrivals();
    std ::cout
        << "Iteration " << i << " took "
        << std::chrono::duration_cast<std::chrono::milliseconds>(delta).count()
        << " ms\n";
    acoustics::utils::printVector(arrivalVec);
  }
  // bhc::writeout(context.params(), context.outputs(), runName);

  return 0;
}
