[TOC]
# Documentation
C++ Documentation can be found at [Docs](https://umich-robotexploration.github.io/manta-ray/)

# Build

## Manta Ray

CMAKE has a `MANTA_CUDA` variable that can be set to determine if CMAKE will look for the `*cudalib.so` or for
`*cxxlib.so`. Initially compiling and linking against the cxx library is much easier and is the recommended path.
Utilizing the cuda build requires ensuring all cuda development tools are installed and functioning.
See [BellhopCuda Permalink](https://github.com/A-New-BellHope/bellhopcuda/blob/b396d40ba49c2f349258b9687cfae8ff8323828f/doc/compilation.md)
for installation instructions utilized when this repository was developed. Note this is a permalink, new instructions
maybe found in the repo on main.

Evaluating initial basic performance, the CUDA version runs a single receiver and source at around 30-60 ms while the
CPU version is taking around ~120 ms. The pain of building and installing CUDA toolkit along with ensuring CMAKE finds
CUDA make be worth it depending on use case.

### Debugging

For debug builds, the debug compiled Bellhopcuda shared library been manually renamed to `*cudalib_debug.so` or
`*cxxlib_debug.so` in the CMAKE. The CMAKE looks for these names, but it is easy to change if needed. You must compile
these debug builds yourself by using the appropriate debug flags in the bellhopcuda project.

## BellhopCuda

In order to build and utilize this project, clone bellhopcuda into a separate location and follow the non-cuda build
instructions.
The generated lib*.so file should be placed in `deps/lib/`.

### CUDA Compilation

Outside the compilation in the instructions of the repository.
I needed to do the following to ensure compilation. Otherwise, I got CMAKE errors. 

> [!NOTE]
> For compilation CMAKE version 3.31.8 was utilized and was found to work over older ROS bundled versions

For **Tim**, need to use `cmake-new` as I
have the incorrectly bundled cmake from ROS.

Additional useful commands that helped compilation succeed but are not covered in the documentation.
```bash
export CUDACXX=/usr/local/cuda/bin/nvcc
cmake .. build -DBHC_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=native
```

# Documentation Generation

To generate the documentation, utilized the `create_docs.sh` bash script. Documentation will be created in the following
folder tree. Additionally, a Github workflow has been setup to make the docs available via a page.

```
|- docs
    |- awesome-doxygen
    |- html
        |- index.html <-- file to open
```



