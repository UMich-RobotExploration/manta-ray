# Define the executable target from the main source file
add_executable(${TARGET_NAME}
        main.cpp)

set(INCLUDE_BHC ${CMAKE_SOURCE_DIR}/deps/bellhopcuda/include/)
set(INCLUDE_GLM ${CMAKE_SOURCE_DIR}/deps/bellhopcuda/glm/)
# Add the bellhopcuda include directory to the project
target_include_directories(${TARGET_NAME} PRIVATE
        ${INCLUDE_BHC})
target_include_directories(${TARGET_NAME} PRIVATE
        ${INCLUDE_GLM})

add_subdirectory(acoustics/)

# Link the acoustics library to the main target
target_link_libraries(${TARGET_NAME} PRIVATE ${ACOUSTIC_LIB_NAME})

# Apply compiler options based on build type
target_compile_options(${TARGET_NAME} PRIVATE
    "$<$<CONFIG:DEBUG>:${DEBUG_COMPILER_OPTIONS}>"
    "$<$<CONFIG:RELEASE>:${RELEASE_COMPILER_OPTIONS}>"
)

