// Grid3D.h
#pragma once

#include <vector>
#include <stdexcept>
#include <sstream>

namespace acoustics {

template<bool Is3D, typename T>
class Grid;

// 2D Grid Specialization
template<typename T>
class Grid<false, T> {
public:
    std::vector<double> xCoords;
    std::vector<double> yCoords;
    std::vector<T> data;

    Grid() = delete;
    Grid(const Grid&) = delete;
    Grid& operator=(const Grid&) = delete;
    Grid(Grid&&) noexcept = delete;
    Grid& operator=(Grid&&) = delete;

    Grid(std::vector<double> x, std::vector<double> y, T defaultValue = T{})
        : xCoords(std::move(x)), yCoords(std::move(y)),
          data(xCoords.size() * yCoords.size(), defaultValue) {
        validateInitialization();
    }

    size_t nx() const { return xCoords.size(); }
    size_t ny() const { return yCoords.size(); }
    size_t size() const { return data.size(); }

    size_t index(size_t ix, size_t iy) const {
        return ix * yCoords.size() + iy;
    }

    T& at(size_t ix, size_t iy) {
        boundsCheck(ix, iy);
        return data[index(ix, iy)];
    }

    const T& at(size_t ix, size_t iy) const {
        boundsCheck(ix, iy);
        return data[index(ix, iy)];
    }

    T& operator()(size_t ix, size_t iy) {
        return data[index(ix, iy)];
    }

    const T& operator()(size_t ix, size_t iy) const {
        return data[index(ix, iy)];
    }

    bool isValid() const {
        return data.size() == xCoords.size() * yCoords.size() &&
               !xCoords.empty() && !yCoords.empty();
    }

private:
    void validateInitialization() const {
        if (xCoords.empty() || yCoords.empty()) {
            throw std::runtime_error("Grid cannot have empty coordinate vectors");
        }
        if (data.size() != xCoords.size() * yCoords.size()) {
            throw std::runtime_error("Grid data size mismatch");
        }
    }

    void boundsCheck(size_t ix, size_t iy) const {
        if (ix >= nx() || iy >= ny()) {
            std::stringstream msg;
            msg << "Grid index out of bounds: (" << ix << ", " << iy
                << ") for grid (" << nx() << ", " << ny() << ")";
            throw std::out_of_range(msg.str());
        }
    }
};

// 3D Grid Specialization
template<typename T>
class Grid<true, T> {
public:
    std::vector<double> xCoords;
    std::vector<double> yCoords;
    std::vector<double> zCoords;
    std::vector<T> data;

    Grid() = delete;
    Grid(const Grid&) = delete;
    Grid& operator=(const Grid&) = delete;
    Grid(Grid&&) noexcept = delete;
    Grid& operator=(Grid&&) = delete;

    Grid(std::vector<double> x, std::vector<double> y, std::vector<double> z,
         T defaultValue = T{})
        : xCoords(std::move(x)), yCoords(std::move(y)), zCoords(std::move(z)),
          data(xCoords.size() * yCoords.size() * zCoords.size(), defaultValue) {
        validateInitialization();
    }

    size_t nx() const { return xCoords.size(); }
    size_t ny() const { return yCoords.size(); }
    size_t nz() const { return zCoords.size(); }
    size_t size() const { return data.size(); }

    size_t index(size_t ix, size_t iy, size_t iz) const {
        return (ix * yCoords.size() + iy) * zCoords.size() + iz;
    }

    T& at(size_t ix, size_t iy, size_t iz) {
        boundsCheck(ix, iy, iz);
        return data[index(ix, iy, iz)];
    }

    const T& at(size_t ix, size_t iy, size_t iz) const {
        boundsCheck(ix, iy, iz);
        return data[index(ix, iy, iz)];
    }

    T& operator()(size_t ix, size_t iy, size_t iz) {
        return data[index(ix, iy, iz)];
    }

    const T& operator()(size_t ix, size_t iy, size_t iz) const {
        return data[index(ix, iy, iz)];
    }

    bool isValid() const {
        return data.size() == xCoords.size() * yCoords.size() * zCoords.size() &&
               !xCoords.empty() && !yCoords.empty() && !zCoords.empty();
    }

private:
    void validateInitialization() const {
        if (xCoords.empty() || yCoords.empty() || zCoords.empty()) {
            throw std::runtime_error("Grid cannot have empty coordinate vectors");
        }
        if (data.size() != xCoords.size() * yCoords.size() * zCoords.size()) {
            throw std::runtime_error("Grid data size mismatch");
        }
    }

    void boundsCheck(size_t ix, size_t iy, size_t iz) const {
        if (ix >= nx() || iy >= ny() || iz >= nz()) {
            std::stringstream msg;
            msg << "Grid index out of bounds: (" << ix << ", " << iy << ", " << iz
                << ") for grid (" << nx() << ", " << ny() << ", " << nz() << ")";
            throw std::out_of_range(msg.str());
        }
    }
};

// Type aliases for convenience
template<typename T>
using Grid2D = Grid<false, T>;

template<typename T>
using Grid3D = Grid<true, T>;

} // namespace acoustics