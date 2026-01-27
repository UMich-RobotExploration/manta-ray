cmake_minimum_required(VERSION 3.20)
project(mantaray LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_VERBOSE_MAKEFILE ON)

set(TARGET_NAME mantaray_core)
set(ACOUSTIC_LIB_NAME acoustics)
# TODO: DISABLE THIS FOR RELEASE BUILDS
# Enable debugging symbols and disable optimizations for debugging
set(CMAKE_BUILD_TYPE Debug)
# For santizing builds -fsanitize=undefined
set(MY_DEBUG_OPTIONS "-Wall;-Wextra;-Wpedantic;-g;-O0")


add_subdirectory(src)

find_library(BELLHPCXXLIB NAMES libbellhopcxxlib.so PATHS ${CMAKE_SOURCE_DIR}/deps/lib)
if (BELLHPCXXLIB)
    message(STATUS "Found Bell HPCXX library: ${BELLHPCXXLIB}")
    target_link_libraries(${ACOUSTIC_LIB_NAME} PUBLIC ${BELLHPCXXLIB})
    #    target_link_libraries(ACOUSTICS PRIVATE ${BELLHPCXXLIB})
else ()
    message(FATAL_ERROR "Could not find Bell HPCXX library!")
endif ()

find_package(Eigen3 REQUIRED)
target_link_libraries(${TARGET_NAME} PUBLIC Eigen3::Eigen)

find_package(manif REQUIRED)
target_include_directories(${TARGET_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})
# Optionally, you can also specify CXX flags specifically for debugging
# If you want extra debug options, you could also do something like:
#target_compile_options(${TARGET_NAME} PUBLIC "$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")