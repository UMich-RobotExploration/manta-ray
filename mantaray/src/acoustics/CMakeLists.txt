add_library(${ACOUSTIC_LIB_NAME}
        Arrival.cpp
        AcousticsBuilder.cpp
        Grid.cpp
        helpers.cpp
)
target_precompile_headers(${ACOUSTIC_LIB_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/acoustics/pch.h)

target_link_libraries(${ACOUSTIC_LIB_NAME} PRIVATE spdlog::spdlog)

# Link the bellhop library to acoustics
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${INCLUDE_BHC})
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${INCLUDE_GLM})
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)  # For assert.h
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${ACOUSTICS_INCLUDE_DIRS}) # Library Includes

# Apply compiler options based on build type
target_compile_options(${ACOUSTIC_LIB_NAME} PUBLIC
        "$<$<CONFIG:Debug>:${DEBUG_COMPILER_OPTIONS}>"
        "$<$<CONFIG:Release>:${RELEASE_COMPILER_OPTIONS}>"
)

