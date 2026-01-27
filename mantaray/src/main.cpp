
#include <atomic>
#include <cstring>
#include <iostream>

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

int main() {
  auto init = bhc::bhcInit();

  std::cout << "Current path is " << std::filesystem::current_path()
            << std::endl;
  // init.FileRoot = "manual";
  init.FileRoot = nullptr;
  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "A");
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
    context.params().ssp->z[i] = 0.0 + static_cast<float>(i) / 10.0 * 20.0;
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

  context.params().ssp->dirty = true;

  std::cout << "Run type: " << context.params().Beam->RunType << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->bot.NPts.x << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->top.NPts.y << "\n";
  try {
    bhc::echo(context.params());
  } catch (const std::exception &e) {
    std::cerr << "Error during echo: " << e.what() << std::endl;
    return 1;
  }
  bhc::run(context.params(), context.outputs());
  // auto arrival = acoustics::Arrival(context.params(), context.outputs());
  // arrival.extractEarliestArrivals();

  bhc::writeout(context.params(), context.outputs(), "manual_mod");

  return 0;
}
