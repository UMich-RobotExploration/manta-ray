# Define the executable target from the main source file
add_executable(${TARGET_NAME}
        main.cpp
        checkAssert.h)


# Add the bellhopcuda include directory to the project
target_include_directories(${TARGET_NAME} PRIVATE
        ${INCLUDE_BHC})
target_include_directories(${TARGET_NAME} PRIVATE
        ${INCLUDE_GLM})
target_include_directories(${TARGET_NAME} PRIVATE
        ${ACOUSTICS_INCLUDE_DIRS})

add_subdirectory(acoustics/)

# Link the acoustics library to the main target
target_link_libraries(${TARGET_NAME} PRIVATE ${ACOUSTIC_LIB_NAME})

# Apply compiler options based on build type
target_compile_options(${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:${DEBUG_COMPILER_OPTIONS}>"
    "$<$<CONFIG:Release>:${RELEASE_COMPILER_OPTIONS}>"
)

# BUILD CONFIGURATIONS
# Enable faster instruction sets (SIMD/AVX)
set(ENABLE_VECTORIZATION ON CACHE BOOL "Enable vectorized instruction sets (SIMD/AVX)? [enabled by default]")

if(${ENABLE_VECTORIZATION})
    message(STATUS "Enabling SIMD/AVX instruction sets")
    add_definitions(-march=native)
endif()
