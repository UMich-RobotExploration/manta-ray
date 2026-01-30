
#include <atomic>
#include <cstring>
#include <iostream>
#include <vector>

#define BHC_DLL_IMPORT 1
#include "acoustics/Arrival.h"
#include "acoustics/BhHandler.h"
#include "acoustics/BoundaryBuilder.h"
#include "acoustics/SspBuilder.h"
#include "acoustics/acousticsConstants.h"
#include "acoustics/helpers.h"

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
  strcpy(context.params().Title, "testing_bath");

  //////////////////////////////////////////////////////////////////////////////
  // Bathymetry Setup
  //////////////////////////////////////////////////////////////////////////////
  auto boundaryBuild = acoustics::BoundaryBuilder(context.params());
  const bhc::IORI2<true> grid = {100, 100};
  const int32_t nBottomProvince = 1;
  boundaryBuild.setupBathymetry(grid, nBottomProvince);
  const double depth = 500;
  std::vector<double> gridX = acoustics::linspace<double>(-10, 10, grid[0]);
  std::vector<double> gridY = acoustics::linspace<double>(-11, 11, grid[1]);
  boundaryBuild.setQuadraticBottom(depth, gridX, gridY, true);
  boundaryBuild.setInterpolationType(acoustics::BathyInterpolationType::kLinear,
                                     false);

  auto output = boundaryBuild.validate();
  acousticsResult.merge(output);

  //////////////////////////////////////////////////////////////////////////////
  // Source / Receivers Setup
  //////////////////////////////////////////////////////////////////////////////
  bhc::extsetup_sxsy(context.params(), 1, 1);
  bhc::extsetup_sz(context.params(), 1);
  context.params().Pos->SxSyInKm = false;
  context.params().Pos->Sx[0] = 10.0f;
  context.params().Pos->Sy[0] = 30.0f;
  context.params().Pos->Sz[0] = 100.0f;

  // disk of receivers
  context.params().Pos->RrInKm = false;
  context.params().Pos->SxSyInKm = false;
  bhc::extsetup_rcvrbearings(context.params(), 5);
  SetupVector(context.params().Pos->theta, 2.0, 20.0, 5);
  bhc::extsetup_rcvrranges(context.params(), 2);
  SetupVector(context.params().Pos->Rr, 10.0, 300.0, 2);
  bhc::extsetup_rcvrdepths(context.params(), 1);
  context.params().Pos->Rz[0] = 50.0;

  //////////////////////////////////////////////////////////////////////////////
  // SSP Setup
  //////////////////////////////////////////////////////////////////////////////
  auto sspBuilder = acoustics::SspBuilder(context.params());
  int nX = 10;
  int nY = 10;
  int nZ = 10;
  sspBuilder.setupHexahedral(nX, nY, nZ, true);
  gridX = acoustics::linspace(-10.0, 10.0, nX);
  gridY = acoustics::linspace(-10.0, 10.0, nY);
  auto gridZ = acoustics::linspace(0.0, 1000.0, nZ);
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
    return 1;
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
  context.params().Beam->Box.z = 1000.0;

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
