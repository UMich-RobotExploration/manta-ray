/*
bellhopcxx / bellhopcuda - C++/CUDA port of BELLHOP(3D) underwater acoustics simulator
Copyright (C) 2021-2023 The Regents of the University of California
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

#include <iostream>
#include <vector>
#include <cstring>

// This define must be set before including the header if you're using the DLL
// version on Windows, and it must NOT be set if you're using the static library
// version on Windows. If you're not on Windows, it doesn't matter either way.
#define BHC_DLL_IMPORT 1
#include <bhc/bhc.hpp>
#include "acoustics/Arrival.h"
#include "acoustics/BhHandler.h"

// Test for the province code.
// Should match the provinces test in the source matlab.
// Also an example for how to run transmission loss in
// a simple environment from library interface.
// Not intended to be general.

void OutputCallback(const char *message)
{
    std::cout << "Out: " << message << std::endl << std::flush;
}

void PrtCallback(const char *message) { std::cout << message << std::flush; }

// Helper to setup vectors from ranges. No checking.
template<class T> void SetupVector(T *arr, T low, T high, int size)
{
    for(int i = 0; i < size; ++i) {
        arr[i] = low + double(i) / double(size - 1) * (high - low);
    }
}

// Helper to setup boundaries. Not very general.
void FlatBoundary3D(
    bhc::BdryInfoTopBot<true> &Boundary, const double &Depth,
    const std::vector<double> &GridX, const std::vector<double> &GridY)
{
    Boundary.dirty     = true;
    Boundary.rangeInKm = true;
    Boundary.NPts[0]   = (int)GridX.size();
    Boundary.NPts[1]   = (int)GridY.size();

    for(auto iy = 0; iy < GridY.size(); ++iy) {
        for(auto ix = 0; ix < GridX.size(); ++ix) {
            Boundary.bd[ix * GridY.size() + iy].x.x = GridX[ix];
            Boundary.bd[ix * GridY.size() + iy].x.y = GridY[iy];
            Boundary.bd[ix * GridY.size() + iy].x.z = Depth;
        }
    }
}

// Hacked in map for the provinces to match a local test. Note province is 1 based.
int GetProvince(int ix, int iy)
{
    if(ix < 5) {
        if(iy < 10) {
            return 3;
        } else {
            return 2;
        }
    } else {
        if(iy < 10) {
            return 4;
        } else {
            return 1;
        }
    }

    // shouldn't get here
}

int main()
{
    // bhc::bhcParams<true> params;
    // bhc::bhcOutputs<true, true> outputs;
    bhc::bhcInit init;
    init.FileRoot       = nullptr;
    init.outputCallback = OutputCallback;
    init.prtCallback    = PrtCallback;

    auto bhContext = acoustics::BhContext<true, true>(init);
    strcpy(bhContext.params().Title, "library province test");

    strcpy(bhContext.params().Beam->RunType, "A");

    // awkward -- freq0 and freqVec are both used and not coordinated.
    bhContext.params().freqinfo->freq0      = 100.0;
    bhContext.params().freqinfo->freqVec[0] = bhContext.params().freqinfo->freq0;

    // flat sound speed
    bhContext.params().ssp->NPts = 3;
    bhContext.params().ssp->Nz   = 3;
    bhContext.params().ssp->z[0] = 0.0;
    bhContext.params().ssp->z[1] = 100.0;
    bhContext.params().ssp->z[2] = 20000.0;
    for(int32_t i = 0; i < 3; ++i) {
        bhContext.params().ssp->alphaR[i] = 1500.0;
        bhContext.params().ssp->alphaI[i] = 0.0;
        bhContext.params().ssp->rho[i]    = 1.0;
        bhContext.params().ssp->betaR[i]  = 0.0;
        bhContext.params().ssp->betaI[i]  = 0.0;
    }
    bhContext.params().ssp->dirty = true;

    // bottom bhContext.params(), mostly overridden, but makes the output file.
    memcpy(bhContext.params().Bdry->Bot.hs.Opt, "A~    ", 6);
    bhContext.params().Bdry->Bot.hs.bc     = 'A';
    bhContext.params().Bdry->Bot.hsx.Sigma = 0.0;
    bhContext.params().Bdry->Bot.hsx.zTemp = 20000.0;
    bhContext.params().Bdry->Bot.hs.alphaR = 1600.0;
    bhContext.params().Bdry->Bot.hs.betaR  = 0.0;
    bhContext.params().Bdry->Bot.hs.rho    = 1.8;
    bhContext.params().Bdry->Bot.hs.alphaI = 0.8;
    bhContext.params().Bdry->Bot.hs.betaI  = 0.0;
    bhContext.params().Bdry->Bot.hs.Depth  = 20000.0;

    // flat top and bottom, variable bottom sound speed
    bhc::extsetup_altimetry(bhContext.params(), {2, 2});
    FlatBoundary3D(bhContext.params().bdinfo->top, 0.0, {-10.0, 10.0}, {-10.0, 10.0});
    bhc::extsetup_bathymetry(bhContext.params(), {11, 21}, 5);
    FlatBoundary3D(
        bhContext.params().bdinfo->bot, 100.0,
        {-10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0},
        {-10.0, -9.0, -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0,
         1.0,   2.0,  3.0,  4.0,  5.0,  6.0,  7.0,  8.0,  9.0,  10.0});
    for(int ix = 0; ix < 11; ++ix) {
        for(int iy = 0; iy < 21; ++iy) {
            bhContext.params().bdinfo->bot.bd[ix * 21 + iy].Province = GetProvince(ix, iy);
        }
    }
    int cnt = 0;
    for(auto spd : {1500.0, 1550.0, 1600.0, 1650.0, 1650.0}) {
        bhContext.params().bdinfo->bot.BotProv[cnt].alphaR = spd;
        bhContext.params().bdinfo->bot.BotProv[cnt].betaR  = 0.0;
        bhContext.params().bdinfo->bot.BotProv[cnt].rho    = 1.8;
        bhContext.params().bdinfo->bot.BotProv[cnt].alphaI = 0.8;
        bhContext.params().bdinfo->bot.BotProv[cnt].betaI  = 0.0;
        ++cnt;
    }
    memcpy(bhContext.params().bdinfo->bot.type, "RL", 2);

    bhContext.params().bdinfo->top.dirty = true;
    bhContext.params().bdinfo->bot.dirty = true;

    // one source
    bhc::extsetup_sxsy(bhContext.params(), 1, 1);
    bhc::extsetup_sz(bhContext.params(), 1);
    bhContext.params().Pos->Sx[0] = 0.0f;
    bhContext.params().Pos->Sy[0] = 0.0f;
    bhContext.params().Pos->Sz[0] = 50.0f;

    // disk of receivers
    bhContext.params().Pos->RrInKm = true;
    bhc::extsetup_rcvrbearings(bhContext.params(), 2);
    SetupVector(bhContext.params().Pos->theta, 2.0, 10.0, 2);
    bhc::extsetup_rcvrranges(bhContext.params(), 2);
    SetupVector(bhContext.params().Pos->Rr, 10.0, 15.0, 2);
    bhc::extsetup_rcvrdepths(bhContext.params(), 1);
    bhContext.params().Pos->Rz[0] = 100.0;

    // source beams
    bhContext.params().Angles->alpha.inDegrees = true;
    bhContext.params().Angles->beta.inDegrees  = true;
    bhc::extsetup_raybearings(bhContext.params(), 144);
    SetupVector(bhContext.params().Angles->beta.angles, 0.0, 10.0, 144);
    bhc::extsetup_rayelevations(bhContext.params(), 200);
    SetupVector(bhContext.params().Angles->alpha.angles, -14.66, 20.0, 200);

    bhContext.params().Beam->rangeInKm = true;
    bhContext.params().Beam->deltas    = 0.0;
    bhContext.params().Beam->Box.x     = 9.99;
    bhContext.params().Beam->Box.y     = 9.99;
    bhContext.params().Beam->Box.z     = 20500.0;

    bhc::echo(bhContext.params());
    bhc::run(bhContext.params(), bhContext.outputs());

    // generate the .env and associated files
    bhc::writeenv(bhContext.params(), "test_province");

    // save the shd file for external use
    bhc::writeout(bhContext.params(), bhContext.outputs(), "test_province");

    auto arrivals = acoustics::Arrival(bhContext.params(), bhContext.outputs().arrinfo);
    arrivals.extractEarliestArrivals();

    return 0;
}
