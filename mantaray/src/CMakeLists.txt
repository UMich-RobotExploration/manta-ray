# Define the executable target from the main source file
add_executable(${TARGET_NAME}
        province.cpp)

# Add the bellhopcuda include directory to the project
target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/deps/bellhopcuda/include/)
target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/deps/bellhopcuda/glm/)

