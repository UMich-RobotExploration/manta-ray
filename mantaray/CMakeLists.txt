cmake_minimum_required(VERSION 3.20)
project(mantaray LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TARGET_NAME mantaray_core)

add_subdirectory(src)

find_library(BELLHPCXXLIB NAMES libbellhopcudalib.so PATHS ${CMAKE_SOURCE_DIR}/deps/lib)
if (BELLHPCXXLIB)
    message(STATUS "Found Bell HPCXX library: ${BELLHPCXXLIB}")
    target_link_libraries(${TARGET_NAME} PRIVATE ${BELLHPCXXLIB})
else()
    message(FATAL_ERROR "Could not find Bell HPCXX library!")
endif()

find_package(Eigen3 REQUIRED)
target_link_libraries(${TARGET_NAME} PUBLIC Eigen3::Eigen)

find_package(manif REQUIRED)
target_include_directories(${TARGET_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})