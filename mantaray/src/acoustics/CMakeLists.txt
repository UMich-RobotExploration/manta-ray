add_library(${ACOUSTIC_LIB_NAME}
        Arrival.cpp
        Arrival.h
        BhHandler.h
        acousticsConstants.h
        helpers.h
        ValidationResult.h
        BoundaryBuilder.cpp
        BoundaryBuilder.h
        SspBuilder.cpp
        SspBuilder.h
        BeamBuilder.cpp
        BeamBuilder.h
        SourceReceiverBuilder.cpp
        SourceReceiverBuilder.h
        BellhopValidator.cpp
        BellhopValidator.h
)

# Link the bellhop library to acoustics
target_include_directories(${ACOUSTIC_LIB_NAME} PRIVATE ${INCLUDE_BHC})
target_include_directories(${ACOUSTIC_LIB_NAME} PRIVATE ${INCLUDE_GLM})

# Apply compiler options based on build type
target_compile_options(${ACOUSTIC_LIB_NAME} PUBLIC
    "$<$<CONFIG:DEBUG>:${DEBUG_COMPILER_OPTIONS}>"
    "$<$<CONFIG:RELEASE>:${RELEASE_COMPILER_OPTIONS}>"
)

