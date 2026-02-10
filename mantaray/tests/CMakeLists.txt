
# These tests can use the Catch2-provided main
add_executable(tests test_helpers.cpp)
target_link_libraries(tests PUBLIC ${ACOUSTIC_LIB_NAME} PRIVATE Catch2::Catch2WithMain)
target_include_directories(tests PUBLIC ${ACOUSTICS_INCLUDE_DIRS}) # Library Includes
# Apply compiler options based on build type
target_compile_options(tests PUBLIC
        "$<$<CONFIG:Debug>:${DEBUG_COMPILER_OPTIONS}>"
        "$<$<CONFIG:Release>:${RELEASE_COMPILER_OPTIONS}>"
)