
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

// #define BHC_DLL_IMPORT 1
#include "acoustics/Arrival.h"
#include "acoustics/BhHandler.h"
#include "acoustics/Result.h"
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
  auto acousticsResult = acoustics::Result();

  char runName[] = "overhaul";
  std::cout << "Current path is " << std::filesystem::current_path()
            << std::endl;
  // init.FileRoot = "manual";
  init.FileRoot = nullptr;
  init.prtCallback = PrtCallback;
  init.outputCallback = OutputCallback;
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
  acoustics::SSPConfig sspConfig = acoustics::SSPConfig{
      acoustics::Grid3D<double>(SSPgridX, SPPgridY, SSPgridZ, 1500.0), true};
  // std::cout << sspConfig.Grid.at(0, 0, 0) << "\n";

  //////////////////////////////////////////////////////////////////////////////
  // Source / Receivers Setup
  //////////////////////////////////////////////////////////////////////////////
  Eigen::Vector3d sourcePos(10.0, 0.0, 100.0);
  Eigen::Vector3d receiverPos;
  receiverPos(0) = 500;
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

  //////////////////////////////////////////////////////////////////////////////
  // Beam Setup
  //////////////////////////////////////////////////////////////////////////////
  // context.params().Angles->alpha.inDegrees = true;
  // context.params().Angles->beta.inDegrees = true;
  // bhc::extsetup_raybearings(context.params(), 30);
  // acoustics::utils::unsafeSetupVector(context.params().Angles->beta.angles,
  //                                     -10.0, 12.0, 30);
  // bhc::extsetup_rayelevations(context.params(), 30);
  // acoustics::utils::unsafeSetupVector(context.params().Angles->alpha.angles,
  //                                     -20.0, 50.0, 30);
  //
  // context.params().Beam->rangeInKm = true;
  // // Here we need to define the limit of the beam tracing to prevent
  // exceeding
  // // the bathymetry grid and the SSP grid
  // context.params().Beam->deltas = 0.01;
  // context.params().Beam->Box.x = 4.0;
  // context.params().Beam->Box.y = 4.0;
  // context.params().Beam->Box.z = bathDepth * 1000 + 40.0 / 1000.0;
  // context.params().freqinfo->Nfreq = 1;
  // context.params().freqinfo->freq0 = 1000.0;
  // bhc::extsetup_sbp(context.params(), 1);
  // context.params().sbp->SBPFlag = 'O';  // Enable source beam pattern
  // context.params().sbp->SBPIndB = true;
  // context.params().sbp->NSBPPts = 1;
  //
  // // context.params().sbp->SrcBmPat[0] = 200.0; // dB

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

  for (int i = 0; i < 10; ++i) {
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();
    bhc::run(context.params(), context.outputs());
    std::chrono::nanoseconds delta =
        std::chrono::high_resolution_clock::now() - t1;
    auto &agentConfig = simBuilder.getAgentsConfig();
    agentConfig.receivers(0) += 50;
    simBuilder.updateAgents();
    // // Debugging Ray memory issues
    // std::cout << "Max points per ray: "
    //           << context.outputs().rayinfo->MaxPointsPerRay << "\n";
    // std::cout << "Num Arrays : " << context.outputs().rayinfo->NRays << "\n";
    // std::cout << "Array Mem Capacity : "
    //           << context.outputs().rayinfo->RayMemCapacity << "\n";
    //
    // std::cout << "\n" << context.outputs().rayinfo->NRays << " rays:\n";
    // for (int r = 0; r < context.outputs().rayinfo->NRays; ++r) {
    //   std::cout << "\nRay " << r << ", " <<
    //   context.outputs().rayinfo->results[r].Nsteps
    //             << "steps, SrcDeclAngle = "
    //             << context.outputs().rayinfo->results[r].SrcDeclAngle <<
    //             ":\n";
    //   for(int s=0; s<context.outputs().rayinfo->results[r].Nsteps; ++s){
    //     std::cout << context.outputs().rayinfo->results[r].ray[s] << "\n";
    //   }
    // }

    // bhc::postprocess(context.params(), context.outputs());
    // bhc::writeout(context.params(), context.outputs(), runName);

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
