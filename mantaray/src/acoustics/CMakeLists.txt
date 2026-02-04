add_library(${ACOUSTIC_LIB_NAME}
        Arrival.cpp
        Arrival.h
        BhHandler.h
        acousticsConstants.h
        helpers.h
        SimulationConfig.h
        Grid.h
        AcousticsBuilder.cpp
        AcousticsBuilder.h
        ../checkAssert.h
)

# Link the bellhop library to acoustics
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${INCLUDE_BHC})
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${INCLUDE_GLM})
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(${ACOUSTIC_LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)  # For assert.h

# Apply compiler options based on build type
target_compile_options(${ACOUSTIC_LIB_NAME} PUBLIC
    "$<$<CONFIG:Debug>:${DEBUG_COMPILER_OPTIONS}>"
    "$<$<CONFIG:Release>:${RELEASE_COMPILER_OPTIONS}>"
)

