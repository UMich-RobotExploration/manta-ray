#include <iostream>
#include <string>

#include <bhc/bhc.hpp>

void PrtCallback(const char *msg) { std::cout << msg << std::flush; }
void OutputCallback(const char *msg) { std::cerr << msg << std::endl; }

template <bool O3D, bool R3D>
int run(const std::string &fileRoot, const char *runTypeOverride) {
  bhc::bhcParams<O3D> params;
  bhc::bhcOutputs<O3D, R3D> outputs;
  bhc::bhcInit init;
  init.FileRoot = fileRoot.c_str();
  init.prtCallback = PrtCallback;
  init.outputCallback = OutputCallback;

  if (!bhc::setup(init, params, outputs))
    return 1;

  if (runTypeOverride) {
    params.Beam->RunType[0] = runTypeOverride[0];
  }

  if (!bhc::run(params, outputs))
    return 1;
  if (!bhc::writeout(params, outputs, fileRoot.c_str()))
    return 1;

  bhc::finalize(params, outputs);
  return 0;
}

int main(int argc, char **argv) {
  int dim = 0;
  const char *runType = nullptr;
  std::string fileRoot;

  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    if (s == "--2D")
      dim = 2;
    else if (s == "--3D")
      dim = 3;
    else if (s == "--run-type" && i + 1 < argc)
      runType = argv[++i];
    else if (s[0] != '-')
      fileRoot = s;
    else {
      std::cerr << "Unknown option: " << s << "\n";
      return 1;
    }
  }

  if (fileRoot.empty()) {
    std::cerr << "Usage: " << argv[0]
              << " [--2D|--3D] [--run-type R] <FileRoot>\n"
              << "FileRoot is path without .env extension\n";
    return 1;
  }
  if (dim == 0) {
    std::cerr << "Specify --2D or --3D\n";
    return 1;
  }

  if (dim == 2)
    return run<false, false>(fileRoot, runType);
  if (dim == 3)
    return run<true, true>(fileRoot, runType);
  return 1;
}
