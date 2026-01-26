add_library(ACOUSTICS Arrival.cpp Arrival.h)

# Link the bellhop library to acoustics
target_include_directories(ACOUSTICS PRIVATE ${INCLUDE_BHC})
target_include_directories(ACOUSTICS PRIVATE ${INCLUDE_GLM})
