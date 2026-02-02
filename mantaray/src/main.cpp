
#include <atomic>
#include <cstring>
#include <iostream>
#include <vector>

#define BHC_DLL_IMPORT 1
#include "acoustics/Agents.h"
#include "acoustics/Arrival.h"
#include "acoustics/BhHandler.h"
#include "acoustics/BoundaryBuilder.h"
#include "acoustics/SspBuilder.h"
#include "acoustics/acousticsConstants.h"
#include "acoustics/helpers.h"

#include "acoustics/Grid.h"
#include "acoustics/SimulationBuilder.h"
#include "acoustics/SimulationConfig.h"
#include <bhc/bhc.hpp>
#include <filesystem>

// Helper to setup vectors from ranges. No checking.
template <class T> void SetupVector(T *arr, T low, T high, int size) {
  for (int i = 0; i < size; ++i) {
    arr[i] = low + double(i) / double(size - 1) * (high - low);
  }
}

void PrtCallback(const char *message) { std::cout << message << std::flush; }
void OutputCallback(const char *message) {
  std::cout << "Out: " << message << std::endl << std::flush;
}

int main() {
  auto init = bhc::bhcInit();
  auto acousticsResult = acoustics::Result();

  std::cout << "Current path is " << std::filesystem::current_path()
            << std::endl;
  // init.FileRoot = "manual";
  init.FileRoot = nullptr;
  init.prtCallback = PrtCallback;
  init.outputCallback = OutputCallback;
  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "R");
  strcpy(context.params().Title, "overhaul");

  //////////////////////////////////////////////////////////////////////////////
  // Bathymetry Setup
  //////////////////////////////////////////////////////////////////////////////

  // std::vector<double> bathGridX = acoustics::utils::linspace<double>(-10, 10, grid[0]);
  // std::vector<double> bathGridY = acoustics::utils::linspace<double>(-11, 11, grid[1]);
  // acoustics::Grid2D<double> bathGrid = acoustics::Grid2D<double>(
  //     bathGridX, bathGridY, );
  // acoustics::BathymetryConfig bathConfig = acoustics::BathymetryConfig(
  //     bathGrid, acoustics::BathyInterpolationType::kLinear, false);

  auto boundaryBuild = acoustics::BoundaryBuilder(context.params());
  const bhc::IORI2<true> grid = {100, 100};
  const int32_t nBottomProvince = 1;
  boundaryBuild.setupBathymetry(grid, nBottomProvince);
  const double bathDepth = 800;
  std::vector<double> gridX = acoustics::utils::linspace<double>(-10, 10, grid[0]);
  std::vector<double> gridY = acoustics::utils::linspace<double>(-11, 11, grid[1]);
  boundaryBuild.setQuadraticBottom(bathDepth, gridX, gridY, true);
  boundaryBuild.setInterpolationType(acoustics::BathyInterpolationType::kLinear,
                                     false);

  auto output = boundaryBuild.validate();
  acousticsResult.merge(output);

  //////////////////////////////////////////////////////////////////////////////
  // Source / Receivers Setup
  //////////////////////////////////////////////////////////////////////////////
  auto agents = acoustics::Agents(context.params());
  agents.initializeSource(10.0, 30.0, 100.0, false);
  // bhc::extsetup_sxsy(context.params(), 1, 1);
  // bhc::extsetup_sz(context.params(), 1);
  // context.params().Pos->SxSyInKm = false;
  // context.params().Pos->Sx[0] = 10.0f;
  // context.params().Pos->Sy[0] = 30.0f;
  // context.params().Pos->Sz[0] = 100.0f;

  // disk of receivers
  std::vector<double> x = acoustics::utils::linspace(100.0, 500.0, 10);
  std::vector<double> y = acoustics::utils::linspace(-10.0, 10.0, 10);
  std::vector<float> z(y.size(), 50.0f);
  acousticsResult.merge(agents.initializeReceivers(x, y, z, false));
  if (acousticsResult.err()) {
    acousticsResult.print();
    return 1;
  }
  // context.params().Pos->RrInKm = false;
  // context.params().Pos->SxSyInKm = false;
  // bhc::extsetup_rcvrbearings(context.params(), 5);
  // SetupVector(context.params().Pos->theta, 2.0, 20.0, 5);
  // bhc::extsetup_rcvrranges(context.params(), 2);
  // SetupVector(context.params().Pos->Rr, 10.0, 300.0, 2);
  // bhc::extsetup_rcvrdepths(context.params(), 1);
  // context.params().Pos->Rz[0] = 50.0;

  //////////////////////////////////////////////////////////////////////////////
  // SSP Setup
  //////////////////////////////////////////////////////////////////////////////
  auto sspBuilder = acoustics::SspBuilder(context.params());
  int nX = 10;
  int nY = 10;
  int nZ = 10;
  sspBuilder.setupHexahedral(nX, nY, nZ, true);
  gridX = acoustics::utils::linspace(-10.0, 10.0, nX);
  gridY = acoustics::utils::linspace(-10.0, 10.0, nY);
  auto gridZ = acoustics::utils::linspace(0.0, 2000.0, nZ);
  auto coordResult = sspBuilder.setCoordinateGrid(gridX, gridY, gridZ);
  if (coordResult.err()) {
    acousticsResult.merge(coordResult);
    acousticsResult.print();
    return 1;
  }
  sspBuilder.setConstantSsp(1500.0);
  acousticsResult.merge(sspBuilder.validate());
  if (acousticsResult.hasWarnings()) {
    acousticsResult.print();
  }

  //////////////////////////////////////////////////////////////////////////////
  // Beam Setup
  //////////////////////////////////////////////////////////////////////////////
  context.params().Angles->alpha.inDegrees = true;
  context.params().Angles->beta.inDegrees = true;
  bhc::extsetup_raybearings(context.params(), 144);
  SetupVector(context.params().Angles->beta.angles, 0.0, 12.0, 144);
  bhc::extsetup_rayelevations(context.params(), 20);
  SetupVector(context.params().Angles->alpha.angles, -14.66, 20.0, 20);

  context.params().Beam->rangeInKm = true;
  // Here we need to define the limit of the beam tracing to prevent exceeding
  // the bathymetry grid and the SSP grid
  context.params().Beam->deltas = 1.0;
  context.params().Beam->Box.x = 7.0;
  context.params().Beam->Box.y = 7.0;
  context.params().Beam->Box.z = bathDepth + 10.0;

  std::cout << "Run type: " << context.params().Beam->RunType << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->bot.NPts.x << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->top.NPts.y << "\n";
  if (acousticsResult.err()) {
    acousticsResult.print();
  } else if (acousticsResult.hasWarnings()) {
    acousticsResult.print();
  }
  try {
    bhc::echo(context.params());
  } catch (const std::exception &e) {
    std::cerr << "Error during echo: " << e.what() << std::endl;
    return 1;
  }
  bhc::writeenv(context.params(), "testing_bath");
  bhc::run(context.params(), context.outputs());
  // auto arrival = acoustics::Arrival(context.params(), context.outputs());
  // arrival.extractEarliestArrivals();

  // bhc::writeout(context.params(), context.outputs(), "testing_bath");

  return 0;
}
