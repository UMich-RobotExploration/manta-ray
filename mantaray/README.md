# Build
## Manta Ray
The CMAKE has a `MANTA_CUDA` variable that can be set to determine if CMAKE will look for the `*cudalib.so` or for `*cxxlib.so`.
Evaluating initial basic performance, the CUDA version runs a single receiver and source at around 30-60 ms while the CPU version is taking around ~120 ms. The pain of building and installing CUDA toolkit along with ensuring CMAKE finds CUDA make be worth it depending on use case.

### Debugging
For debug builds, the debug compiled Bellhopcuda shared library been manually renamed to `*cudalib_debug.so` or `*cxxlib_debug.so` in the CMAKE. The CMAKE looks for these names, but it is easy to change if needed. You must compile these debug builds yourself by using the appropriate debug flags in the bellhopcuda project. 

## BellhopCuda
In order to build and utilize this project, clone bellhopcuda into a separate location and follow the non-cuda build instructions.
The generated lib*.so file should be placed in `deps/lib/`. 

### CUDA Compilation

Outside of the compilation in the instructions of the repository. 
I needed to do the following to ensure compilation. Otherwise I got CMAKE errors. For *TIM* need to use `cmake-new` as I have the incorrectly bundled cmake from ROS. My `cmake-new --version = 3.31.8` for future contributors and users. Other commands I needed to add to find CUDA appropriately:

```bash
export CUDACXX=/usr/local/cuda/bin/nvcc
cmake .. build -DBHC_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=native
```


