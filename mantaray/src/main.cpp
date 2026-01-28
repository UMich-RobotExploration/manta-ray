
#include <atomic>
#include <cstring>
#include <iostream>
#include <vector>

#define BHC_DLL_IMPORT 1
#include "acoustics/Arrival.h"
#include "acoustics/BhHandler.h"
#include <bhc/bhc.hpp>
#include <filesystem>

// Helper to setup vectors from ranges. No checking.
template <class T> void SetupVector(T *arr, T low, T high, int size) {
  for (int i = 0; i < size; ++i) {
    arr[i] = low + double(i) / double(size - 1) * (high - low);
  }
}

void PrtCallback(const char *message) { std::cout << message << std::flush; }

void FlatBoundary3D(bhc::BdryInfoTopBot<true> &Boundary, const double &Depth,
                    const std::vector<double> &GridX,
                    const std::vector<double> &GridY) {
  Boundary.dirty = true;
  Boundary.rangeInKm = true;
  Boundary.NPts[0] = static_cast<int>(GridX.size());
  Boundary.NPts[1] = static_cast<int>(GridY.size());

  for (auto iy = 0; iy < GridY.size(); ++iy) {
    for (auto ix = 0; ix < GridX.size(); ++ix) {
      Boundary.bd[ix * GridY.size() + iy].x.x = GridX[ix];
      Boundary.bd[ix * GridY.size() + iy].x.y = GridY[iy];
      Boundary.bd[ix * GridY.size() + iy].x.z = Depth;
    }
  }
}

void QuadBoundary3D(bhc::BdryInfoTopBot<true> &Boundary, const double &Depth,
                    const std::vector<double> &GridX,
                    const std::vector<double> &GridY) {
  Boundary.dirty = true;
  Boundary.rangeInKm = true;
  Boundary.NPts[0] = static_cast<int>(GridX.size());
  Boundary.NPts[1] = static_cast<int>(GridY.size());
  for (auto iy = 0; iy < GridY.size(); ++iy) {
    for (auto ix = 0; ix < GridX.size(); ++ix) {
      Boundary.bd[ix * GridY.size() + iy].x.x = GridX[ix];
      Boundary.bd[ix * GridY.size() + iy].x.y = GridY[iy];
      Boundary.bd[ix * GridY.size() + iy].x.z =
          Depth - std::pow(GridX[ix] / 10, 2.0) - std::pow(GridY[iy] / 10, 2.0);
      // PROVINCE IS 1 INDEXED
      Boundary.bd[ix * GridY.size() + iy].Province = 1;
    }
  }
}

int main() {
  auto init = bhc::bhcInit();

  std::cout << "Current path is " << std::filesystem::current_path()
            << std::endl;
  // init.FileRoot = "manual";
  init.FileRoot = nullptr;
  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "A");
  strcpy(context.params().Title, "testing_bath");

  //////////////////////////////////////////////////////////////////////////////
  // SSP Setup
  //////////////////////////////////////////////////////////////////////////////
  bhc::extsetup_ssp_hexahedral(context.params(), 10, 10, 10);
  context.params().ssp->Nx = 10;
  context.params().ssp->Ny = 10;
  context.params().ssp->Nz = 10;
  context.params().ssp->NPts = 10;
  context.params().ssp->rangeInKm = true;
  // coords in params.ssp->Seg.x, .y, .z, and the speeds
  // * in params.ssp->cMat[(x*Ny+y)*Nz+z].
  for (auto i = 0; i < 10; ++i) {
    context.params().ssp->Seg.x[i] =
        -10.0 + static_cast<float>(i) / 10.0 * 20.0;
    context.params().ssp->Seg.y[i] =
        -10.0 + static_cast<float>(i) / 10.0 * 20.0;
    context.params().ssp->Seg.z[i] = 0.0 + static_cast<float>(i) / 10.0 * 20.0;
    context.params().ssp->z[i] = 0.0 + static_cast<float>(i) / 10.0 * 10.0;
  }
  for (auto i = 0; i < 10; ++i) {
    for (auto j = 0; j < 10; ++j) {
      for (auto k = 0; k < 10; ++k) {
        context.params().ssp->cMat[(i * 10 + j) * 10 + k] = 1500.0;
      }
    }
  }
  //  After writing your SSP, make sure params.Bdry->Top.hs.Depth (nominal
  //  surface depth, normally zero) is equal to ssp->Seg.z[0], and
  //  params.Bdry->Bot.hs.Depth (nominal ocean bottom depth) is equal to
  //  ssp->Seg.z[Nz-1].
  context.params().Bdry->Top.hs.Depth = context.params().ssp->Seg.z[0];
  context.params().Bdry->Bot.hs.Depth = context.params().ssp->Seg.z[10 - 1];
  std::cout << "Maximum depth of SSP Profile "
            << context.params().ssp->Seg.z[10 - 1] << "\n";

  context.params().ssp->dirty = true;

  //////////////////////////////////////////////////////////////////////////////
  // Bathymetry Setup
  //////////////////////////////////////////////////////////////////////////////

  const bhc::IORI2<true> grid = {5, 5};
  const int32_t nBottomProvince = 1;
  bhc::extsetup_bathymetry(context.params(), grid, nBottomProvince);
  const double depth = context.params().ssp->Seg.z[10 - 1];
  std::vector<double> gridX = {-5, -3, 0, 3, 5};
  std::vector<double> gridY = {-5, -3, 0, 3, 5};
  QuadBoundary3D(context.params().bdinfo->bot, depth, gridX, gridY);

  //////////////////////////////////////////////////////////////////////////////
  // Top altimetry setup
  //////////////////////////////////////////////////////////////////////////////
  bhc::extsetup_altimetry(context.params(), {2, 2});
  FlatBoundary3D(context.params().bdinfo->top, 0.0, {-10.0, 10.0},
                 {-10.0, 10.0});
  //////////////////////////////////////////////////////////////////////////////
  // Source / Receivers Setup
  //////////////////////////////////////////////////////////////////////////////
  bhc::extsetup_sxsy(context.params(), 1, 1);
  bhc::extsetup_sz(context.params(), 1);
  context.params().Pos->SxSyInKm = false;
  context.params().Pos->Sx[0] = 0.1f;
  context.params().Pos->Sy[0] = 0.1f;
  context.params().Pos->Sz[0] = 10.0f;

  // disk of receivers
  context.params().Pos->RrInKm = false;
  bhc::extsetup_rcvrbearings(context.params(), 5);
  SetupVector(context.params().Pos->theta, 2.0, 20.0, 5);
  bhc::extsetup_rcvrranges(context.params(), 2);
  SetupVector(context.params().Pos->Rr, 10.0, 300.0, 2);
  bhc::extsetup_rcvrdepths(context.params(), 1);
  context.params().Pos->Rz[0] = 50.0;

  //////////////////////////////////////////////////////////////////////////////
  // Beam Setup
  //////////////////////////////////////////////////////////////////////////////
  context.params().Angles->alpha.inDegrees = true;
  context.params().Angles->beta.inDegrees = true;
  bhc::extsetup_raybearings(context.params(), 144);
  SetupVector(context.params().Angles->beta.angles, 0.0, 12.0, 144);
  bhc::extsetup_rayelevations(context.params(), 200);
  SetupVector(context.params().Angles->alpha.angles, -14.66, 20.0, 200);

  context.params().Beam->rangeInKm = false;
  // Here we need to define the limit of the beam tracing to prevent exceeding
  // the bathymetry grid and the SSP grid
  context.params().Beam->deltas = 0.0;
  context.params().Beam->Box.x = 5.0;
  context.params().Beam->Box.y = 5.0;
  context.params().Beam->Box.z = 5.0;

  std::cout << "Run type: " << context.params().Beam->RunType << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->bot.NPts.x << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->top.NPts.y << "\n";
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

  bhc::writeout(context.params(), context.outputs(), "manual_mod");

  return 0;
}
