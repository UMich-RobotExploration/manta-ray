#define CATCH_CONFIG_MAIN
#pragma once
#include <catch2/catch_all.hpp>
#include <Eigen/Dense>
#include <vector>
#include <thread>
#include <manif/SE3.h>  // Manif SE3 includes
#include "rb/Integrator.h"  // Assuming the header with integratePose is included

constexpr int NUM_ENTITIES = 30;  // Number of entities to process
const double dt = 0.1;          // Time step for twist * dt = pos operation
constexpr double mass = 100.0;        // Mass for wrench / m = accel operation

// Typedef for 6D Vector (twist, wrench, and combined accelerations)
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> MassMatrix;

struct EntityAOS {
    manif::SE3Tangentd twist;        // Combined twist (linear and angular)
    manif::SE3d pose;      // SE3 object for position and orientation
    Vector6d wrench;       // Combined wrench (force and torque)
    Vector6d accel;        // Combined acceleration (linear and angular)
    MassMatrix massMatrix; // Mass matrix (6x6)
};

// Data structure for SOA (Structure of Arrays)
struct SOA {
    std::vector<manif::SE3Tangentd> twist;    // Combined twist (linear and angular)
    std::vector<manif::SE3d> pose; // SE3 object for position and orientation
    std::vector<Vector6d> wrench;  // Combined wrench (force and torque)
    std::vector<Vector6d> accel;   // Combined acceleration (linear and angular)
    std::vector<MassMatrix> massMatrix; // Mass matrix (6x6)
};

// Helper function to initialize random data for Vector6d elements
void initialize_data(std::vector<Vector6d>& data) {
    for (auto& vec : data) {
        vec = Vector6d::Random();  // Random 6D vector (range [-1, 1])
    }
}
// Helper function to initialize random data for Vector6d elements
void initialize_data(std::vector<manif::SE3Tangentd>& data) {
  for (auto& vec : data) {
    vec = Vector6d::Random();  // Random 6D vector (range [-1, 1])
  }
}

// Helper function to initialize random data for SE3 elements (position + quaternion)
void initialize_data(std::vector<manif::SE3d>& data) {
    for (auto& se3 : data) {
        se3 = manif::SE3d::Random();  // Generate random SE3 object (position + orientation)
    }
}

// Helper function to initialize random data for MassMatrix elements
void initialize_data(std::vector<MassMatrix>& data) {
    for (auto& mat : data) {
        mat.setZero();

        // Generate a random matrix B (3x3) and construct a positive semi-definite matrix
        Eigen::Matrix3d B = Eigen::Matrix3d::Random();
        mat.topLeftCorner<3, 3>() = B * B.transpose();  // Ensure positive semi-definiteness by making it B*B^T

        // Set the bottom-right part as m * I (mass * identity)
        mat.bottomRightCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
    }
}

// Helper function to initialize data for AOS (Array of Structures)
void initialize_data(std::vector<EntityAOS>& entities) {
    for (auto& entity : entities) {
        entity.twist = Vector6d::Random();     // Initialize twist (linear + angular)
        entity.wrench = Vector6d::Random();    // Initialize wrench (force + torque)
        entity.pose = manif::SE3d::Random();   // Initialize SE3 (pose)
        entity.accel = Vector6d::Random();     // Initialize combined acceleration
        entity.massMatrix.setZero();           // Initialize mass matrix
        Eigen::Matrix3d B = Eigen::Matrix3d::Random();
        entity.massMatrix.topLeftCorner<3, 3>() = B * B.transpose();  // Random positive semi-definite inertia matrix
        entity.massMatrix.bottomRightCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();  // Mass * I
    }
}

// Dummy operation for AOS (Array of Structures)
inline void update_AOS(std::vector<EntityAOS>& entities) {
    for (auto& entity : entities) {
        // Integrate pose based on twist and time step using the integratePose function
        rb::integratePose(entity.pose, entity.twist, dt, entity.pose);

        // Integrate velocity and acceleration using the mass matrix and wrench
        rb::integrateVel(entity.massMatrix, entity.wrench, entity.twist, dt, entity.twist, entity.accel);
    }
}

// Dummy operation for SOA (Structure of Arrays)
inline void update_SOA(SOA& soa) {
    for (int i = 0; i < NUM_ENTITIES; ++i) {
        // Integrate pose based on twist and time step using the integratePose function
        rb::integratePose(soa.pose[i], soa.twist[i], dt, soa.pose[i]);

        // Integrate velocity and acceleration using the mass matrix and wrench
        rb::integrateVel(soa.massMatrix[i], soa.wrench[i], soa.twist[i], dt, soa.twist[i], soa.accel[i]);
    }
}

inline void update_AOS_parallel(std::vector<EntityAOS>& entities) {
  const int numThreads = 4;  // Use the number of available cores (or a smaller number if performance drops)
  std::vector<std::thread> threads;
  int chunkSize = NUM_ENTITIES / numThreads;

  // Create threads and assign work
  for (int t = 0; t < numThreads; ++t) {
    threads.push_back(std::thread([&, t]() {
        int startIdx = t * chunkSize;
        int endIdx = (t == numThreads - 1) ? NUM_ENTITIES : (t + 1) * chunkSize;

        for (int i = startIdx; i < endIdx; ++i) {
          auto entity = entities[i];
          // Integrate pose based on twist and time step using the integratePose function
          rb::integratePose(entity.pose, entity.twist, dt, entity.pose);

          // Integrate velocity and acceleration using the mass matrix and wrench
          rb::integrateVel(entity.massMatrix, entity.wrench, entity.twist, dt, entity.twist, entity.accel);
          }
    }));
  }

  // Wait for threads to finish
  for (auto& t : threads) {
    t.join();
  }
}

inline void update_SOA_parallel(SOA& soa) {
  const int numThreads = 4;  // Use the number of available cores (or a smaller number if performance drops)
  std::vector<std::thread> threads;
  int chunkSize = NUM_ENTITIES / numThreads;

  // Create threads and assign work
  for (int t = 0; t < numThreads; ++t) {
    threads.push_back(std::thread([&, t]() {
        int startIdx = t * chunkSize;
        int endIdx = (t == numThreads - 1) ? NUM_ENTITIES : (t + 1) * chunkSize;

        for (int i = startIdx; i < endIdx; ++i) {
            rb::integratePose(soa.pose[i], soa.twist[i], dt, soa.pose[i]);
            rb::integrateVel(soa.massMatrix[i], soa.wrench[i], soa.twist[i], dt, soa.twist[i], soa.accel[i]);
        }
    }));
  }

  // Wait for threads to finish
  for (auto& t : threads) {
    t.join();
  }
}


TEST_CASE("Benchmark AOS vs SOA", "[benchmark]") {
    // ========================
    // AOS Test
    // ========================
    std::vector<EntityAOS> entitiesAOS(NUM_ENTITIES);

    // Initialize with random data
    initialize_data(entitiesAOS);

    BENCHMARK("AOS update") {
        update_AOS(entitiesAOS);
    };

    // ========================
    // SOA Test
    // ========================
    SOA soa;
    soa.twist.resize(NUM_ENTITIES);
    soa.pose.resize(NUM_ENTITIES);
    soa.wrench.resize(NUM_ENTITIES);
    soa.accel.resize(NUM_ENTITIES);
    soa.massMatrix.resize(NUM_ENTITIES);

    // Initialize SOA with random data
    initialize_data(soa.twist);  // Initialize the twist (linear + angular)
    initialize_data(soa.wrench); // Initialize the wrench (force + torque)
    initialize_data(soa.pose);   // Initialize SE3 (pose)
    initialize_data(soa.massMatrix);  // Initialize mass matrices

    BENCHMARK("SOA update") {
        update_SOA(soa);
    };

  BENCHMARK("AOS parallel update") {
    update_AOS_parallel(entitiesAOS);
  };

  BENCHMARK("SOA parallel update") {
    update_SOA_parallel(soa);
  };
}
