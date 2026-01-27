add_library(${ACOUSTIC_LIB_NAME} Arrival.cpp Arrival.h)

# Link the bellhop library to acoustics
target_include_directories(${ACOUSTIC_LIB_NAME} PRIVATE ${INCLUDE_BHC})
target_include_directories(${ACOUSTIC_LIB_NAME} PRIVATE ${INCLUDE_GLM})
target_compile_options(${ACOUSTIC_LIB_NAME} PUBLIC "$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")
