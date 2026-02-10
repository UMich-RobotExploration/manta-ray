//
// Created by tko on 2/10/26.
//

#include "rb/pch.h"

#include "rb/Integrator.h"
#include <catch2/catch_test_macros.hpp>

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
  auto twist = manif::SE3Tangentd().setZero();
  auto poseOutput = manif::SE3d().setIdentity();

  auto pose = manif::SE3d(Eigen::Vector3d(0.0, 0.0, 0.0),
                          Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));

  Eigen::Vector3d angVel(0.8, 0, 0);
  twist.ang() = angVel;
  double dt = 1.0;

  rb::integratePose(pose, twist, dt, poseOutput);

  auto R_initial = pose.rotation();
  auto R_bodyFrame =
      Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitX()).toRotationMatrix();
  REQUIRE(R_initial.isApprox(pose.rotation(), 1e-6));
  auto expected = (R_bodyFrame * R_initial);

  CAPTURE(static_cast<Eigen::AngleAxisd>(expected).angle());
  CAPTURE(static_cast<Eigen::AngleAxisd>(expected).axis().transpose());
  CAPTURE(static_cast<Eigen::AngleAxisd>(poseOutput.rotation()).angle());
  CAPTURE(
      static_cast<Eigen::AngleAxisd>(poseOutput.rotation()).axis().transpose());
  Eigen::Quaterniond quat(expected);
  CAPTURE(quat);
  CAPTURE(poseOutput);
  CHECK(poseOutput.rotation().isApprox(expected, 1e-6));

  // Translation should be unchanged (no linear velocity)
  CAPTURE(poseOutput.translation().transpose());
  CHECK(poseOutput.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0),
                                          1e-6));
}
