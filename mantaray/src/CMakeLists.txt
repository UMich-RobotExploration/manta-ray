# Define the executable target from the main source file
add_executable(${TARGET_NAME}
        main.cpp
        checkAssert.h
        )


# Add the bellhopcuda include directory to the project
target_include_directories(${TARGET_NAME} PRIVATE
        ${INCLUDE_BHC})
target_include_directories(${TARGET_NAME} PRIVATE
        ${INCLUDE_GLM})
target_include_directories(${TARGET_NAME} PRIVATE
        ${ACOUSTICS_INCLUDE_DIRS})

add_subdirectory(acoustics/)
add_subdirectory(rb/)

# Link the acoustics library to the main target
target_link_libraries(${TARGET_NAME} PRIVATE ${ACOUSTIC_LIB_NAME})
target_link_libraries(${TARGET_NAME} PRIVATE ${RB_LIB_NAME})

# sudo apt install libfmt-dev
find_package(fmt)
target_link_libraries(${TARGET_NAME} PRIVATE fmt::fmt)

# Apply compiler options based on build type
target_compile_options(${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:${DEBUG_COMPILER_OPTIONS}>"
    "$<$<CONFIG:Release>:${RELEASE_COMPILER_OPTIONS}>"
)

