cmake_minimum_required(VERSION 3.20)
project(mantaray LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_VERBOSE_MAKEFILE ON)

set(TARGET_NAME mantaray_core)
set(ACOUSTIC_LIB_NAME acoustics)
# Compiler options split by build type
# Debug: Include debugging symbols, no optimizations, all warnings
set(DEBUG_COMPILER_OPTIONS "-Wall;-Wextra;-Wpedantic;-g;-O0")
# For sanitizing builds add: -fsanitize=undefined

# Release: Optimizations enabled, no debug symbols, all warnings
set(RELEASE_COMPILER_OPTIONS "-Wall;-Wextra;-Wpedantic;-O3;-DNDEBUG")

# Print which configuration is being applied
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "${TARGET_NAME}: Using CONFIG:Debug with options: ${DEBUG_COMPILER_OPTIONS}")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "${TARGET_NAME}: Using CONFIG:Release with options: ${RELEASE_COMPILER_OPTIONS}")
else()
    message(STATUS "${TARGET_NAME}: Build type is '${CMAKE_BUILD_TYPE}'")
endif()


set(EIGEN3_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/deps/eigen)

add_subdirectory(src)

find_library(BELLHPCXXLIB NAMES libbellhopcudalib.so PATHS ${CMAKE_SOURCE_DIR}/deps/lib)
if (BELLHPCXXLIB)
    message(STATUS "Found Bell HPCXX library: ${BELLHPCXXLIB}")
    target_link_libraries(${ACOUSTIC_LIB_NAME} PUBLIC ${BELLHPCXXLIB})
    #    target_link_libraries(ACOUSTICS PRIVATE ${BELLHPCXXLIB})
else ()
    message(FATAL_ERROR "Could not find Bell HPCXX library!")
endif ()

#find_package(Eigen3 REQUIRED)
#target_link_libraries(${TARGET_NAME} PUBLIC Eigen3::Eigen)
target_include_directories(${TARGET_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})

#find_package(manif REQUIRED)
#target_include_directories(${TARGET_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})
# Optionally, you can also specify CXX flags specifically for debugging
# If you want extra debug options, you could also do something like:
#target_compile_options(${TARGET_NAME} PUBLIC "$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")
