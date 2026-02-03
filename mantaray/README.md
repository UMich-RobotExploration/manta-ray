# Build
## BellhopCuda
In order to build utilize this project, clone bellhopcuda into a seperate location and follow the non-cuda build instructions.
The generated lib*.so file should be placed in `deps/lib/`

### CUDA Compilation

Outside of the compilation in the instructions of the repository. 
I needed to do the following to ensure compilation. Otherwise I got CMAKE errors. For *TIM* need to use `cmake-new` as I have the incorrectly bundled cmake from ROS. `cmake-new --version = 3.31.8` for future followers.

```bash
export CUDACXX=/usr/local/cuda/bin/nvcc
```
```bash
cmake .. build -DBHC_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=native
```


