#pragma once

#include <iostream>
#include <cstdlib>

// Always-active check that works in both debug and release builds
// Similar to assert() but never disabled by NDEBUG
#define CHECK(condition, message) \
    do { \
        if (!(condition)) { \
            std::cerr << "File: " << __FILE__ << ":" << __LINE__ << "\n" \
                      << "\t" << "Check failed: " << #condition << "\n" \
                      << "\t" << "Message: " << message << "\n"; \
            std::abort(); \
        } \
    } while (0)
