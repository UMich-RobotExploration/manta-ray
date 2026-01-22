#include <iostream>
#include <bhc/bhc.hpp>

int main() {
    // Test the bellhopcuda library by calling a simple function or using a class.
    try {
        // Assuming you have some simple BellhopCUDA setup, you can do something basic like:
        bhc::Environment env;
        std::cout << "BellhopCUDA environment initialized successfully!" << std::endl;
        
        // You can modify the environment and interact with the library further as needed
        // e.g., env.some_function(); or similar API usage
        
    } catch (const std::exception &e) {
        std::cerr << "Error initializing BellhopCUDA: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

