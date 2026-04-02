//
// Created by tko on 2/10/26.
//

#include "rb/pch.h"

#include "rb/Integrator.h"
#include "rb/PhysicsBodies.h"
#include "rb/RbWorld.h"
#include "rb/RobotsAndSensors.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
using Catch::Approx;

TEST_CASE("Linear Integration of pose", "[integrator]") {
  auto twist = manif::SE3Tangentd().setZero();
  auto pose = manif::SE3d().setIdentity();
  pose.coeffs().head(3) = Eigen::Vector3d(1, 0, 0);
  twist.lin() = Eigen::Vector3d(1, 1, 0); // 1 m/s forward
  auto poseOutput = manif::SE3d().setIdentity();
  rb::integratePose(pose, twist, 1, poseOutput);
  // std::cout << "Pose Output Translation: "
  //           << poseOutput.translation().transpose() << std::endl;
  CHECK(poseOutput.translation().isApprox(Eigen::Vector3d(2, 1, 0), 1e-6));

  twist.lin() = Eigen::Vector3d(0, 1, -1);
  poseOutput = manif::SE3d().setIdentity();
  rb::integratePose(pose, twist, 0.01, poseOutput);
  CHECK(
      poseOutput.translation().isApprox(Eigen::Vector3d(1, 0.01, -0.01), 1e-6));
}

TEST_CASE("Linear Integration of pose at rotation", "[integrator]") {
  auto twist = manif::SE3Tangentd().setZero();
  twist.lin() = Eigen::Vector3d(2, 1, 0);
  auto pose =
      manif::SE3d(Eigen::Vector3d(10, 0, 0),
                  Eigen::AngleAxisd(EIGEN_PI * 0.5, Eigen::Vector3d::UnitZ()));
  manif::SE3d poseOutput = pose;
  rb::integratePose(pose, twist, 1, poseOutput);
  // std::cout << "Pose Output Translation: "
  //           << poseOutput.translation().transpose() << std::endl;
  CAPTURE(pose);
  CAPTURE(poseOutput);
  // Globally we are moving 1 negative in the x-axis and 2 positive in the y axis.
  CHECK(poseOutput.translation().isApprox(Eigen::Vector3d(9, 2, 0), 1e-6));
}

TEST_CASE("Angular Integration of pose", "[integrator]") {
  auto twist = manif::SE3Tangentd().setZero();
  auto pose = manif::SE3d().setIdentity();
  twist.ang() = Eigen::Vector3d(1, 0, 0);
  auto poseOutput = manif::SE3d().setIdentity();
  rb::integratePose(pose, twist, 1, poseOutput);
  // std::cout << "Pose Output rotation: \n " << poseOutput.rotation()
  //           << std::endl;
  CHECK(poseOutput.translation().isApprox(Eigen::Vector3d(0, 0, 0), 1e-6));
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX());
  CHECK(poseOutput.rotation().isApprox(m, 1e-6));

  twist = manif::SE3Tangentd().setZero();
  pose = manif::SE3d(1.0, 10.0, 100.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d angVel(1, 2, 0);
  twist.ang() = angVel;
  double dt = 1.0;
  Eigen::Vector3d theta_vec = angVel * dt;
  double theta = theta_vec.norm();
  Eigen::Vector3d axis = theta_vec / theta;
  rb::integratePose(pose, twist, dt, poseOutput);

  Eigen::Matrix3d expected = Eigen::AngleAxisd(theta, axis).toRotationMatrix();
  CAPTURE(expected);
  CAPTURE(poseOutput.rotation());
  CHECK(poseOutput.rotation().isApprox(expected, 1e-6));
  CHECK(poseOutput.translation().isApprox(Eigen::Vector3d(1.0, 10.0, 100.0),
                                          1e-6));
}

TEST_CASE("Local vs Global Integration of pose check", "[integrator]") {
  /* Here I want to test the following:
  * - Rotation around z axis of 0.5 radians
  * - Local rotation around x-axis then using twist of 0.8 rad/s for 1 second
  */
  auto twistLocal = manif::SE3Tangentd().setZero();
  auto outputPose = manif::SE3d().setIdentity();

  auto poseSpatial = manif::SE3d(Eigen::Vector3d(0.0, 0.0, 0.0),
                          Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));

  Eigen::Vector3d angVel(0.8, 0, 0);
  twistLocal.ang() = angVel;
  double dt = 1.0;

  rb::integratePose(poseSpatial, twistLocal, dt, outputPose);

  auto rInitial = poseSpatial.rotation();
  auto rBodyFrame =
      Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitX()).toRotationMatrix();
  REQUIRE(rInitial.isApprox(poseSpatial.rotation(), 1e-6));
  // this is an intrinsic rotation, we rotate around the new axes so
  // we post multiply the body frame
  auto expected = (rInitial * rBodyFrame);
  CAPTURE(static_cast<Eigen::AngleAxisd>(expected).angle());
  CAPTURE(static_cast<Eigen::AngleAxisd>(expected).axis().transpose());
  CAPTURE(static_cast<Eigen::AngleAxisd>(outputPose.rotation()).angle());
  CAPTURE(
      static_cast<Eigen::AngleAxisd>(outputPose.rotation()).axis().transpose());
  Eigen::Quaterniond quat(expected);
  CAPTURE(quat);
  CAPTURE(outputPose);
  CHECK(outputPose.rotation().isApprox(expected, 1e-6));

  // Translation should be unchanged (no linear velocity)
  CAPTURE(outputPose.translation().transpose());
  CHECK(outputPose.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0),
                                          1e-6));
}

// relativeTransformBodyFrame computes: output = from.inverse() * to
// This gives "the transform from 'from' to 'to' expressed in from's frame"

TEST_CASE("Body frame relative transform - pure translation", "[transform]") {
  // Both poses face +X (identity rotation), separated by 5m in world X.
  // In the body frame of 'from', the displacement is also +X.
  auto from = manif::SE3d(Eigen::Vector3d(1, 0, 0), Eigen::Quaterniond::Identity());
  auto to = manif::SE3d(Eigen::Vector3d(6, 0, 0), Eigen::Quaterniond::Identity());
  manif::SE3d rel;
  rb::relativeTransformBodyFrame(from, to, rel);

  CHECK(rel.translation().isApprox(Eigen::Vector3d(5, 0, 0), 1e-6));
  CHECK(rel.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-6));
}

TEST_CASE("Body frame relative transform - rotated frame", "[transform]") {
  // 'from' is at origin rotated 90° about Z (facing +Y in world).
  // 'to' is at (0, 3, 0) with same rotation.
  // World displacement is (0, 3, 0). But in from's body frame, +Y world
  // is +X body (since the robot faces +Y). So body-frame delta = (3, 0, 0).
  auto rot90z = Eigen::AngleAxisd(EIGEN_PI * 0.5, Eigen::Vector3d::UnitZ());
  auto from = manif::SE3d(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(rot90z));
  auto to = manif::SE3d(Eigen::Vector3d(0, 3, 0), Eigen::Quaterniond(rot90z));
  manif::SE3d rel;
  rb::relativeTransformBodyFrame(from, to, rel);

  CHECK(rel.translation().isApprox(Eigen::Vector3d(3, 0, 0), 1e-6));
  CHECK(rel.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-6));
}

TEST_CASE("Body frame relative transform - with rotation change", "[transform]") {
  // 'from' faces +X, 'to' faces +Y (90° rotation about Z).
  // Both at origin, so translation is zero. The body-frame relative rotation
  // should be the same 90° about Z.
  auto from = manif::SE3d(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  auto rot90z = Eigen::AngleAxisd(EIGEN_PI * 0.5, Eigen::Vector3d::UnitZ());
  auto to = manif::SE3d(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(rot90z));
  manif::SE3d rel;
  rb::relativeTransformBodyFrame(from, to, rel);

  CHECK(rel.translation().isApprox(Eigen::Vector3d(0, 0, 0), 1e-6));
  Eigen::Matrix3d expectedRot = rot90z.toRotationMatrix();
  CHECK(rel.rotation().isApprox(expectedRot, 1e-6));
}

////////////////////////////////////////////////////////////////////////////////
// Sensor recording tests
////////////////////////////////////////////////////////////////////////////////

TEST_CASE("GroundTruthPose records at correct frequency", "[sensors]") {
  rb::RbWorld world{};
  world.simData.dt = 0.1;
  world.createRngEngine(42);

  // GT at 1 Hz → should record every 1 second
  auto idx = world.addRobot<rb::ConstantVelRobot>(rb::ConstantVelConfig{{}, {1, 0, 0}});
  world.robots[idx]->addSensor(
      std::make_unique<rb::GroundTruthPose>(1.0, 100));
  world.dynamicsBodies.setPosition(idx, Eigen::Vector3d(10, 20, 30));

  world.advanceWorld(5.0);

  auto *gt = world.robots[idx]->sensors_[0].get();
  const auto &data = gt->getSensorData();
  const auto &timestamps = gt->getSensorTimesteps();

  // 1 Hz over 5 seconds: records at t=0,1,2,3,4,5 = 6 samples
  // (sensor updates after each integration step and includes endpoint)
  REQUIRE(data.size() == timestamps.size());
  CHECK(data.size() == 6);

  // Each entry is 7 elements: [x, y, z, qx, qy, qz, qw]
  CHECK(data[0].size() == 7);

  // First entry should be near initial position (10, 20, 30)
  CHECK(data[0][0] == Approx(10.0).margin(0.2));
  CHECK(data[0][1] == Approx(20.0).margin(0.2));
  CHECK(data[0][2] == Approx(30.0).margin(0.2));
}

TEST_CASE("PositionalXYOdometry with zero noise matches GT position",
          "[sensors]") {
  rb::RbWorld world{};
  world.simData.dt = 0.1;
  world.createRngEngine(42);

  auto idx = world.addRobot<rb::ConstantVelRobot>(rb::ConstantVelConfig{{}, {1, 0, 0}});
  world.robots[idx]->addSensor(
      std::make_unique<rb::GroundTruthPose>(1.0, 100));
  // Zero-noise odometry at 1 Hz
  world.robots[idx]->addSensor(std::make_unique<rb::PositionalXYOdometry>(
      1.0, 100, std::normal_distribution<double>{0.0, 0.0}));
  world.dynamicsBodies.setPosition(idx, Eigen::Vector3d(0, 0, 0));

  world.advanceWorld(3.0);

  auto *gt = world.robots[idx]->sensors_[0].get();
  auto *odom = world.robots[idx]->sensors_[1].get();
  const auto &gtData = gt->getSensorData();
  const auto &odomData = odom->getSensorData();

  REQUIRE(gtData.size() == odomData.size());

  // Each odom entry is 3 elements
  CHECK(odomData[0].size() == 3);

  // With zero noise, odom position should match GT position
  for (size_t i = 0; i < gtData.size(); ++i) {
    CHECK(odomData[i][0] == Approx(gtData[i][0]).margin(1e-6));
    CHECK(odomData[i][1] == Approx(gtData[i][1]).margin(1e-6));
    CHECK(odomData[i][2] == Approx(gtData[i][2]).margin(1e-6));
  }
}

TEST_CASE("GpsPosition only records near surface", "[sensors]") {
  rb::RbWorld world{};
  world.simData.dt = 0.1;
  world.createRngEngine(42);

  // Robot with zero velocity at depth z=100 (well below surface)
  auto idx = world.addRobot<rb::ConstantVelRobot>(rb::ConstantVelConfig{{}, {0, 0, 0}});
  world.robots[idx]->addSensor(std::make_unique<rb::GpsPosition>(
      1.0, 100, std::normal_distribution<double>{0.0, 0.0},
      std::normal_distribution<double>{0.0, 0.0}, 0.1, 0.0));
  world.dynamicsBodies.setPosition(idx, Eigen::Vector3d(0, 0, 100));

  world.advanceWorld(3.0);

  auto *gps = world.robots[idx]->sensors_[0].get();
  // At z=100, well outside surfaceRange=0.1, should record nothing
  CHECK(gps->getSensorData().empty());
}

TEST_CASE("GpsPosition records when at surface", "[sensors]") {
  rb::RbWorld world{};
  world.simData.dt = 0.1;
  world.createRngEngine(42);

  // Robot at z=0 (at surface), zero velocity
  auto idx = world.addRobot<rb::ConstantVelRobot>(rb::ConstantVelConfig{{}, {0, 0, 0}});
  world.robots[idx]->addSensor(std::make_unique<rb::GpsPosition>(
      1.0, 100, std::normal_distribution<double>{0.0, 0.0},
      std::normal_distribution<double>{0.0, 0.0}, 0.1, 0.0));
  world.dynamicsBodies.setPosition(idx, Eigen::Vector3d(5, 10, 0));

  world.advanceWorld(3.0);

  auto *gps = world.robots[idx]->sensors_[0].get();
  const auto &data = gps->getSensorData();
  // At surface, should record at t=0,1,2,3 (inclusive endpoint)
  CHECK(data.size() == 4);
  // Each entry is 3 elements, zero noise → matches position
  CHECK(data[0].size() == 3);
  CHECK(data[0][0] == Approx(5.0).margin(1e-6));
  CHECK(data[0][1] == Approx(10.0).margin(1e-6));
}