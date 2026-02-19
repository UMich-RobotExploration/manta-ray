# Define the executable target from the main source file
add_executable(${TARGET_NAME}
        main.cpp
        checkAssert.h
        Logger.cpp
        Logger.h
        ConfigReader.cpp
        ConfigReader.h
)


# Add the bellhopcuda include directory to the project
target_include_directories(${TARGET_NAME} PRIVATE
        ${INCLUDE_BHC})
target_include_directories(${TARGET_NAME} PRIVATE
        ${INCLUDE_GLM})
target_include_directories(${TARGET_NAME} PRIVATE
        ${ACOUSTICS_INCLUDE_DIRS})

# sudo apt install libfmt-dev
find_package(fmt REQUIRED)
target_link_libraries(${TARGET_NAME} PRIVATE fmt::fmt)

# sudo apt install libspdlog-dev
find_package(spdlog REQUIRED)
target_link_libraries(${TARGET_NAME} PRIVATE spdlog::spdlog)

add_subdirectory(acoustics/)
add_subdirectory(rb/)

# Link the acoustics library to the main target
target_link_libraries(${TARGET_NAME} PRIVATE ${ACOUSTIC_LIB_NAME})
target_link_libraries(${TARGET_NAME} PRIVATE ${RB_LIB_NAME})


# Apply compiler options based on build type
target_compile_options(${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:${DEBUG_COMPILER_OPTIONS}>"
    "$<$<CONFIG:Release>:${RELEASE_COMPILER_OPTIONS}>"
)

