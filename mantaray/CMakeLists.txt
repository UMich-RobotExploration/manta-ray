cmake_minimum_required(VERSION 3.20)
project(mantaray LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_VERBOSE_MAKEFILE ON)

set(TARGET_NAME mantaray_core)
set(ACOUSTIC_LIB_NAME acoustics)

set(LIBS_DIR ${CMAKE_SOURCE_DIR}/deps)

set(INCLUDE_BHC ${LIBS_DIR}/bellhopcuda/include/)
set(INCLUDE_GLM ${LIBS_DIR}/bellhopcuda/glm/)
set(INCLUDE_ACOUSTICS ${CMAKE_SOURCE_DIR}/src/acoustics/)

# Debug Printing
message(STATUS "Build type: '${CMAKE_BUILD_TYPE}'")
message(STATUS "Config: '$<CONFIG>'")

option(MANTA_CUDA "Enable CUDA" ON)
# Compiler options split by build type
# Debug: Include debugging symbols, no optimizations, all warnings
set(DEBUG_COMPILER_OPTIONS "-Wall;-Wextra;-Wpedantic;-g;-O0")
# For sanitizing builds add: -fsanitize=undefined

# Release: Optimizations enabled, no debug symbols, all warnings
set(RELEASE_COMPILER_OPTIONS "-Wall;-Wextra;-Wpedantic;-O3;-DNDEBUG")

# Here I'm selecting if we should use the CUDA or CXX version of the Bellhop library
if(MANTA_CUDA)
    set(LIB_EXTENSION "cuda")
else()
    set(LIB_EXTENSION "cxx")
endif()

# Print which configuration is being applied
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "${TARGET_NAME}: Using CONFIG:Debug with options: ${DEBUG_COMPILER_OPTIONS}")
    set(BELLHOP_LIB libbellhop${LIB_EXTENSION}lib_debug.so)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "${TARGET_NAME}: Using CONFIG:Release with options: ${RELEASE_COMPILER_OPTIONS}")
    set(BELLHOP_LIB libbellhop${LIB_EXTENSION}lib.so)
else()
    message(STATUS "${TARGET_NAME}: Build type is '${CMAKE_BUILD_TYPE}'")
endif()


set(EIGEN3_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/deps/eigen)

add_subdirectory(src)

find_library(BELLHPCXXLIB NAMES ${BELLHOP_LIB} PATHS ${CMAKE_SOURCE_DIR}/deps/lib)
if (BELLHPCXXLIB)
    message(STATUS "Found Bell HPCXX library: ${BELLHPCXXLIB} from ${BELLHOP_LIB}")
    target_link_libraries(${ACOUSTIC_LIB_NAME} PUBLIC ${BELLHPCXXLIB})
else ()
    message(STATUS "Could not find Bell HPCXX library!")
    message(FATAL_ERROR "Looked for: ${BELLHOP_LIB} in ${CMAKE_SOURCE_DIR}/deps/lib")
endif ()

#find_package(Eigen3 REQUIRED)
#target_link_libraries(${TARGET_NAME} PUBLIC Eigen3::Eigen)
target_include_directories(${TARGET_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})

#find_package(manif REQUIRED)
#target_include_directories(${TARGET_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})
# Optionally, you can also specify CXX flags specifically for debugging
# If you want extra debug options, you could also do something like:
#target_compile_options(${TARGET_NAME} PUBLIC "$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")

# Enable testing only if requested
option(BUILD_TESTS "Build the tests" ON)

if(BUILD_TESTS)
    enable_testing()
    set(CATCH2_DIR ${LIBS_DIR}/Catch2/ CACHE PATH "Path to top-level directory of Catch2 library (the one containing CMakeLists.txt)")
    add_subdirectory(${CATCH2_DIR})
    add_subdirectory(tests)
endif()