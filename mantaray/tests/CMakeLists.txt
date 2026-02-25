
# These tests can use the Catch2-provided main
add_executable(tests test_helpers.cpp test_Integrator.cpp)
target_link_libraries(tests PUBLIC ${ACOUSTIC_LIB_NAME} PRIVATE Catch2::Catch2WithMain)
target_include_directories(tests PRIVATE ${INCLUDE_GLM})
target_include_directories(tests PRIVATE ${INCLUDE_BHC})
target_include_directories(tests PUBLIC ${ACOUSTICS_INCLUDE_DIRS}) # Library Includes
target_link_libraries(tests PUBLIC ${RB_LIB_NAME} PRIVATE Catch2::Catch2WithMain)
target_include_directories(tests PUBLIC ${RB_INCLUDE_DIRS}) # Library Includes
# Apply compiler options based on build type
target_compile_options(tests PUBLIC
        "$<$<CONFIG:Debug>:${DEBUG_COMPILER_OPTIONS}>"
        "$<$<CONFIG:Release>:${RELEASE_COMPILER_OPTIONS}>"
)