#define CATCH_CONFIG_MAIN
#pragma once
#include "rb/Integrator.h"
#include <Eigen/Dense>
#include <catch2/catch_all.hpp>
#include <manif/SE3.h>
#include <thread>
#include <vector>

constexpr int NUM_ENTITIES = 30;
const double dt = 0.1;
constexpr double mass = 100.0;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> MassMatrix;

// AOS structure
struct EntityAOS {
  manif::SE3Tangentd twist;
  manif::SE3d pose;
  Vector6d wrench;
  Vector6d accel;
  MassMatrix massMatrix;
};

// Original SOA structure
struct SOA {
  std::vector<manif::SE3Tangentd> twist;
  std::vector<manif::SE3d> pose;
  std::vector<Vector6d> wrench;
  std::vector<Vector6d> accel;
  std::vector<MassMatrix> massMatrix;
};

struct PhysicsBodiesSOA {
  struct DynamicsProperties{
    Eigen::Matrix<double,6,6> massMatrix = Eigen::Matrix<double,6,6>::Zero();
    double mass = -100.0;
  };

  struct DynamicsData{
    Vector6d wrench = Vector6d::Zero();
    Vector6d accel = Vector6d::Zero();
  };

  struct KinematicData {
    manif::SE3d pose = manif::SE3d::Identity();
    manif::SE3Tangentd twist = manif::SE3Tangentd::Zero();
  };

  std::vector<DynamicsData> dynamics;
  std::vector<KinematicData> kinematics;
  std::vector<DynamicsProperties> properties;
  size_t size() const { return kinematics.size(); }
};

// Helper functions for Vector6d
void initialize_data(std::vector<Vector6d> &data) {
  for (auto &vec : data) {
    vec = Vector6d::Random();
  }
}

void initialize_data(std::vector<manif::SE3Tangentd> &data) {
  for (auto &vec : data) {
    vec = Vector6d::Random();
  }
}

void initialize_data(std::vector<manif::SE3d> &data) {
  for (auto &se3 : data) {
    se3 = manif::SE3d::Random();
  }
}

void initialize_data(std::vector<MassMatrix> &data) {
  for (auto &mat : data) {
    mat.setZero();
    Eigen::Matrix3d B = Eigen::Matrix3d::Random();
    mat.topLeftCorner<3, 3>() = B * B.transpose();
    mat.bottomRightCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
  }
}

void initialize_data(std::vector<EntityAOS> &entities) {
  for (auto &entity : entities) {
    entity.twist = Vector6d::Random();
    entity.wrench = Vector6d::Random();
    entity.pose = manif::SE3d::Random();
    entity.accel = Vector6d::Random();
    entity.massMatrix.setZero();
    Eigen::Matrix3d B = Eigen::Matrix3d::Random();
    entity.massMatrix.topLeftCorner<3, 3>() = B * B.transpose();
    entity.massMatrix.bottomRightCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
  }
}

void initialize_data(PhysicsBodiesSOA &bodies, size_t count) {
  bodies.dynamics.resize(count);
  bodies.kinematics.resize(count);

  for (size_t i = 0; i < count; ++i) {
    bodies.kinematics[i].pose = manif::SE3d::Random();
    bodies.kinematics[i].twist = Vector6d::Random();
    bodies.dynamics[i].wrench = Vector6d::Random();
    bodies.dynamics[i].accel = Vector6d::Random();
    bodies.properties[i].massMatrix.setZero();

    Eigen::Matrix3d B = Eigen::Matrix3d::Random();
    bodies.properties[i].massMatrix.topLeftCorner<3, 3>() = B * B.transpose();
    bodies.properties[i].massMatrix.bottomRightCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
  }
}

// AOS update functions
inline void update_AOS(std::vector<EntityAOS> &entities) {
  for (auto &entity : entities) {
    rb::integratePose(entity.pose, entity.twist, dt, entity.pose);
  }
}


// Original SOA update functions
inline void update_SOA(SOA &soa) {
  for (int i = 0; i < NUM_ENTITIES; ++i) {
    rb::integratePose(soa.pose[i], soa.twist[i], dt, soa.pose[i]);
  }
}


// Hybrid SOA update functions
inline void update_hybrid_SOA(PhysicsBodiesSOA &bodies) {
  for (size_t i = 0; i < bodies.size(); ++i) {
    auto &kin = bodies.kinematics[i];
    auto &dyn = bodies.dynamics[i];

    rb::integratePose(kin.pose, kin.twist, dt, kin.pose);
  }
}


TEST_CASE("Benchmark AOS vs SOA vs Hybrid SOA", "[benchmark]") {
  // ========================
  // AOS Test
  // ========================
  std::vector<EntityAOS> entitiesAOS(NUM_ENTITIES);
  initialize_data(entitiesAOS);

  BENCHMARK("AOS update") { update_AOS(entitiesAOS); };
  // BENCHMARK("AOS parallel update") { update_AOS_parallel(entitiesAOS); };

  // ========================
  // Original SOA Test
  // ========================
  SOA soa;
  soa.twist.resize(NUM_ENTITIES);
  soa.pose.resize(NUM_ENTITIES);
  soa.wrench.resize(NUM_ENTITIES);
  soa.accel.resize(NUM_ENTITIES);
  soa.massMatrix.resize(NUM_ENTITIES);

  initialize_data(soa.twist);
  initialize_data(soa.wrench);
  initialize_data(soa.pose);
  initialize_data(soa.massMatrix);

  BENCHMARK("SOA update") { update_SOA(soa); };
  // BENCHMARK("SOA parallel update") { update_SOA_parallel(soa); };

  // ========================
  // Hybrid SOA Test
  // ========================
  PhysicsBodiesSOA hybridSOA;
  initialize_data(hybridSOA, NUM_ENTITIES);

  BENCHMARK("Hybrid SOA update") { update_hybrid_SOA(hybridSOA); };
  // BENCHMARK("Hybrid SOA parallel update") { update_hybrid_SOA_parallel(hybridSOA); };
}
