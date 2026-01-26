
#include <atomic>
#include <cstring>
#include <iostream>

#define BHC_DLL_IMPORT 1
#include <bhc/bhc.hpp>

void OutputCallback(const char *message) {
  std::cout << "Out: " << message << std::endl << std::flush;
}

void PrtCallback(const char *message) { std::cout << message << std::flush; }

// Detect when the threads all complete control.
std::atomic<bool> going;
void CompletedCallback() {
  std::cout << "CompletedCallback called.\n" << std::flush;
  going = false;
}
/*
std::ostream& operator<<(std::ostream& out, const bhc::rayPt<false>& x) {
    out << x.NumTopBnc << " "
        << x.NumBotBnc << " "
        << x.x.x << " " << x.x.y << " "
        << x.t.x << " " << x.t.y << " "
        << x.p.x << " " << x.p.y << " "
        << x.q.x << " " << x.q.y << " "
        << x.c << " "
        << x.Amp << " " << x.Phase << " "
        << x.tau;
    return out;
}
*/
std::ostream &operator<<(std::ostream &out, const bhc::cpxf &x) {
  out << "(" << x.real() << ", " << x.imag() << ")";
  return out;
}

int main() {
  bhc::bhcParams<false> params;
  bhc::bhcOutputs<false, false> outputs;
  bhc::bhcInit init;
  init.FileRoot = nullptr;
  init.outputCallback = OutputCallback;
  init.prtCallback = PrtCallback;
  init.completedCallback = CompletedCallback;

  bhc::setup(init, params, outputs);

  // setup the beam and bounds
  // strcpy(params.Beam->RunType, "RG   3");
  strcpy(params.Beam->RunType, "R");
  params.Beam->rangeInKm = true;
  params.Beam->deltas = 1.01;
  params.Beam->Box.x = 20.0;

  // receivers for TL, mostly ignored in ray mode
  int num_receivers = 5;
  bhc::extsetup_rcvrranges(params, num_receivers);
  bhc::extsetup_rcvrdepths(params, 1);
  params.Pos->RrInKm = true;
  for (int i = 0; i < num_receivers; ++i) {
    params.Pos->Rr[i] = float(i + 1) / 10.0f;
  }

  int num_rays = 10;
  bhc::extsetup_rayelevations(params, num_rays);
  params.Angles->alpha.inDegrees = true;
  for (int i = 0; i < num_rays; ++i) {
    params.Angles->alpha.angles[i] = -60 + 120 * double(i) / double(num_rays);
  }
  bhc::echo(params);

  bhc::run(params, outputs);

  std::cout << "Post processing: " << bhc::postprocess(params, outputs) << "\n";

  bhc::writeenv(params, "background_ray");
  // bhc::writeout(params, outputs, "background_ray");

  // TODO: Uncomment for printing rays
  // IMPORTANT: Access to ray info through iteration
  // std::cout << "\n" << outputs.rayinfo->NRays << " rays:\n";
  // for(int r=0; r<outputs.rayinfo->NRays; ++r){
  //     std::cout << "\nRay " << r
  //         << ", " << outputs.rayinfo->results[r].Nsteps
  //         << "steps, SrcDeclAngle = " <<
  //         outputs.rayinfo->results[r].SrcDeclAngle
  //         << ":\n";
  //     for(int s=0; s<outputs.rayinfo->results[r].Nsteps; ++s){
  //         auto curr_ray = &outputs.rayinfo->results[r].ray[s];
  //         std::cout << curr_ray->tau << "\n";
  //         // can access individual ray coordinates if we want here
  //         std::cout << "(" << curr_ray->x.r << "," << curr_ray->x.y << ")" <<
  //         "\n";
  //     }
  // }

  bhc::finalize(params, outputs);
  return 0;
}
