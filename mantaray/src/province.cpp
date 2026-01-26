/*
bellhopcxx / bellhopcuda - C++/CUDA port of BELLHOP(3D) underwater acoustics
simulator Copyright (C) 2021-2023 The Regents of the University of California
Marine Physical Lab at Scripps Oceanography, c/o Jules Jaffe, jjaffe@ucsd.edu
Based on BELLHOP / BELLHOP3D, which is Copyright (C) 1983-2022 Michael B. Porter

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <array>
#include <cstring>
#include <iostream>
#include <vector>

// This define must be set before including the header if you're using the DLL
// version on Windows, and it must NOT be set if you're using the static library
// version on Windows. If you're not on Windows, it doesn't matter either way.
#define BHC_DLL_IMPORT 1

#include "acoustics/Arrival.h"
#include <Eigen/Dense>
#include <bhc/bhc.hpp>
#include <manif/manif.h>
// Test for the province code.
// Should match the provinces test in the source matlab.
// Also an example for how to run transmission loss in
// a simple environment from library interface.
// Not intended to be general.

void OutputCallback(const char *message) {
  std::cout << "Out: " << message << std::endl << std::flush;
}

void PrtCallback(const char *message) { std::cout << message << std::flush; }

// Helper to setup vectors from ranges. No checking.
template <class T> void SetupVector(T *arr, T low, T high, int size) {
  for (int i = 0; i < size; ++i) {
    arr[i] = low + double(i) / double(size - 1) * (high - low);
  }
}

// Helper to setup boundaries. Not very general.
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

// Hacked in map for the provinces to match a local test. Note province is 1
// based.
int GetProvince(int ix, int iy) {
  if (ix < 5) {
    if (iy < 10) {
      return 3;
    } else {
      return 2;
    }
  } else {
    if (iy < 10) {
      return 4;
    } else {
      return 1;
    }
  }

  // shouldn't get here
}

int main() {
  bhc::bhcParams<true> params;
  bhc::bhcOutputs<true, true> outputs;
  bhc::bhcInit init;
  init.FileRoot = nullptr;
  init.outputCallback = OutputCallback;
  init.prtCallback = PrtCallback;

  // init.numThreads = 3;
  bhc::setup(init, params, outputs);




  strcpy(params.Title, "library province test");

  strcpy(params.Beam->RunType, "A");

  // awkward -- freq0 and freqVec are both used and not coordinated.
  params.freqinfo->freq0 = 1000.0;
  params.freqinfo->freqVec[0] = params.freqinfo->freq0;

  // flat sound speed
  params.ssp->NPts = 3;
  params.ssp->Nz = 3;
  params.ssp->z[0] = 0.0;
  params.ssp->z[1] = 100.0;
  params.ssp->z[2] = 20000.0;
  for (int32_t i = 0; i < 3; ++i) {
    params.ssp->alphaR[i] = 1500.0;
    params.ssp->alphaI[i] = 0.0;
    params.ssp->rho[i] = 1.0;
    params.ssp->betaR[i] = 0.0;
    params.ssp->betaI[i] = 0.0;
  }
  params.ssp->dirty = true;

  // bottom params, mostly overridden, but makes the output file.
  memcpy(params.Bdry->Bot.hs.Opt, "A~    ", 6);
  params.Bdry->Bot.hs.bc = 'A';
  params.Bdry->Bot.hsx.Sigma = 0.0;
  params.Bdry->Bot.hsx.zTemp = 20000.0;
  params.Bdry->Bot.hs.alphaR = 1600.0;
  params.Bdry->Bot.hs.betaR = 0.0;
  params.Bdry->Bot.hs.rho = 1.8;
  params.Bdry->Bot.hs.alphaI = 0.8;
  params.Bdry->Bot.hs.betaI = 0.0;
  params.Bdry->Bot.hs.Depth = 20000.0;

  // flat top and bottom, variable bottom sound speed
  bhc::extsetup_altimetry(params, {2, 2});
  FlatBoundary3D(params.bdinfo->top, 0.0, {-10.0, 10.0}, {-10.0, 10.0});
  bhc::extsetup_bathymetry(params, {11, 21}, 5);
  FlatBoundary3D(params.bdinfo->bot, 100.0,
                 {-10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0},
                 {-10.0, -9.0, -8.0, -7.0, -6.0, -5.0, -4.0,
                  -3.0,  -2.0, -1.0, 0.0,  1.0,  2.0,  3.0,
                  4.0,   5.0,  6.0,  7.0,  8.0,  9.0,  10.0});
  for (int ix = 0; ix < 11; ++ix) {
    for (int iy = 0; iy < 21; ++iy) {
      params.bdinfo->bot.bd[ix * 21 + iy].Province = GetProvince(ix, iy);
    }
  }
  int cnt = 0;
  for (auto spd : {1500.0, 1550.0, 1600.0, 1650.0, 1650.0}) {
    params.bdinfo->bot.BotProv[cnt].alphaR = spd;
    params.bdinfo->bot.BotProv[cnt].betaR = 0.0;
    params.bdinfo->bot.BotProv[cnt].rho = 1.8;
    params.bdinfo->bot.BotProv[cnt].alphaI = 0.8;
    params.bdinfo->bot.BotProv[cnt].betaI = 0.0;
    ++cnt;
  }
  memcpy(params.bdinfo->bot.type, "RL", 2);

  params.bdinfo->top.dirty = true;
  params.bdinfo->bot.dirty = true;

  // one source
  bhc::extsetup_sxsy(params, 1, 1);
  bhc::extsetup_sz(params, 1);
  params.Pos->Sx[0] = 0.0f;
  params.Pos->Sy[0] = 0.0f;
  params.Pos->Sz[0] = 10.0f;

  // disk of receivers
  params.Pos->RrInKm = true;
  bhc::extsetup_rcvrbearings(params, 2);
  SetupVector(params.Pos->theta, 0.0, 10.0, 2);
  bhc::extsetup_rcvrranges(params, 2);
  SetupVector(params.Pos->Rr, 9.0, 10.0, 2);
  // WARN: So setup requires us to prespecify a number and the iterate thorugh
  // the array and apply the numbers. It is not take a vector as an argument
  bhc::extsetup_rcvrdepths(params, 1);
  params.Pos->Rz[0] = 10.0;

  // source beams
  params.Angles->alpha.inDegrees = true;
  params.Angles->beta.inDegrees = true;

  bhc::extsetup_raybearings(params, 700);
  SetupVector(params.Angles->beta.angles, -5.0, 15.0, 700);
  bhc::extsetup_rayelevations(params, 50);
  SetupVector(params.Angles->alpha.angles, -10.0, 10.0, 50);

  params.Beam->rangeInKm = true;
  params.Beam->deltas = 0;
  params.Beam->Box.x = 9.99;
  params.Beam->Box.y = 9.99;
  params.Beam->Box.z = 2050.0;

  bhc::echo(params);
  bhc::run(params, outputs);

  // generate the .env and associated files
  bhc::writeenv(params, "test_province");
  // strcpy(params.Beam->RunType, "A");
  // bhc::run(params, outputs);
  bhc::postprocess(params, outputs);

  // save the shd file for external use
  // bhc::writeout(params, outputs, "test_province");
  acoustics::Arrival arrival = acoustics::Arrival(&params,(outputs.arrinfo));
  try{
    arrival.extractEarliestArrivals();
  } catch (const std::runtime_error& e) {
    std::cerr << "Error during arrival extraction: " << e.what() << std::endl;
  }





  bhc::finalize(params, outputs);
}