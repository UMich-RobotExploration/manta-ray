# Define the executable target from the main source file
add_executable(mantaray_core main.cpp)

# Add the bellhopcuda include directory to the project
target_include_directories(mantaray_core PRIVATE
        ${CMAKE_SOURCE_DIR}/deps/bellhopcuda/include/bhc/include/)

# Link the shared library (libbellhopcxxlib.so) to the executable
target_link_libraries(mantaray_core
        PRIVATE
        ${CMAKE_SOURCE_DIR}/deps/bellhopcuda/lib/libbellhopcxxlib.so
)
