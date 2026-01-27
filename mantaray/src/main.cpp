
#include <atomic>
#include <cstring>
#include <iostream>

#define BHC_DLL_IMPORT 1
#include "acoustics/Arrival.h"
#include "acoustics/BhHandler.h"
#include <bhc/bhc.hpp>
#include <filesystem>

int main() {
  auto init = bhc::bhcInit();

  std::cout << "Current path is " << std::filesystem::current_path()
            << std::endl;
  init.FileRoot = "manual";
  auto context = acoustics::BhContext<true, true>(init);
  strcpy(context.params().Beam->RunType, "A");
  std::cout << "Run type: " << context.params().Beam->RunType << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->bot.NPts.x << "\n";
  std::cout << "Boundary: " << context.params().bdinfo->top.NPts.y << "\n";
  bhc::run(context.params(), context.outputs());
  // auto arrival = acoustics::Arrival(context.params(), context.outputs());
  // arrival.extractEarliestArrivals();

  bhc::writeout(context.params(), context.outputs(), "manual");

  return 0;
}
