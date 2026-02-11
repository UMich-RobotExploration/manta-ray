add_library(${RB_LIB_NAME}
        PhysicsBodies.cpp
        ConstantVelRobot.cpp
        RobotI.cpp
)
target_precompile_headers(${RB_LIB_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/rb/pch.h)


# Link the bellhop library to acoustics
target_include_directories(${RB_LIB_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(${RB_LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)  # For assert.h
target_include_directories(${RB_LIB_NAME} PUBLIC ${RB_INCLUDE_DIRS}) # Library Includes

find_package(manif REQUIRED)
target_include_directories(${TARGET_NAME} SYSTEM PUBLIC ${manif_INCLUDE_DIRS})

# Apply compiler options based on build type
target_compile_options(${RB_LIB_NAME} PUBLIC
        "$<$<CONFIG:Debug>:${DEBUG_COMPILER_OPTIONS}>"
        "$<$<CONFIG:Release>:${RELEASE_COMPILER_OPTIONS}>"
)

