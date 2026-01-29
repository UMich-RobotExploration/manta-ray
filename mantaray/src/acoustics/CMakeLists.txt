add_library(${ACOUSTIC_LIB_NAME} Arrival.cpp Arrival.h
        BhHandler.h
        simControls.h
        helpers.h)

# Link the bellhop library to acoustics
target_include_directories(${ACOUSTIC_LIB_NAME} PRIVATE ${INCLUDE_BHC})
target_include_directories(${ACOUSTIC_LIB_NAME} PRIVATE ${INCLUDE_GLM})

# Apply compiler options based on build type
target_compile_options(${ACOUSTIC_LIB_NAME} PUBLIC
    "$<$<CONFIG:DEBUG>:${DEBUG_COMPILER_OPTIONS}>"
    "$<$<CONFIG:RELEASE>:${RELEASE_COMPILER_OPTIONS}>"
)

