//
// test_PfgWriter.cpp
//

#include "mantaray/utils/Logger.h"
#include "mantaray/utils/PfgWriter.h"
#include "rb/RbWorld.h"
#include "rb/RobotsAndSensors.h"

#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {
// Read all lines from a file, stripping empty trailing lines
std::vector<std::string> readLines(const std::string &path) {
  std::ifstream f(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(f, line)) {
    lines.push_back(line);
  }
  while (!lines.empty() && lines.back().empty()) {
    lines.pop_back();
  }
  return lines;
}
} // namespace

////////////////////////////////////////////////////////////////////////////////
// Covariance helpers
////////////////////////////////////////////////////////////////////////////////

TEST_CASE("makeDiagUpperTri6x6 places values at correct positions",
          "[pfgwriter]") {
  auto cov = pyfg::makeDiagUpperTri6x6(1, 2, 3, 4, 5, 6);

  // Diagonal positions in 6x6 upper-tri row-major
  CHECK(cov[0] == 1.0);
  CHECK(cov[6] == 2.0);
  CHECK(cov[11] == 3.0);
  CHECK(cov[15] == 4.0);
  CHECK(cov[18] == 5.0);
  CHECK(cov[20] == 6.0);

  // All off-diagonal should be zero
  for (size_t i = 0; i < 21; ++i) {
    if (i != 0 && i != 6 && i != 11 && i != 15 && i != 18 && i != 20) {
      CHECK(cov[i] == 0.0);
    }
  }
}

TEST_CASE("makeDiagUpperTri3x3 places values at correct positions",
          "[pfgwriter]") {
  auto cov = pyfg::makeDiagUpperTri3x3(10, 20, 30);

  CHECK(cov[0] == 10.0);
  CHECK(cov[3] == 20.0);
  CHECK(cov[5] == 30.0);

  CHECK(cov[1] == 0.0);
  CHECK(cov[2] == 0.0);
  CHECK(cov[4] == 0.0);
}

////////////////////////////////////////////////////////////////////////////////
// End-to-end PFG writer output
////////////////////////////////////////////////////////////////////////////////

TEST_CASE("writePfg produces correct section ordering and field counts",
          "[pfgwriter]") {
  init_logger();

  rb::RbWorld world{};
  world.simData.dt = 1.0;
  world.createRngEngine(42);

  // 2 stationary robots with GT at 1 Hz, odom at 1 Hz (zero noise)
  for (int r = 0; r < 2; ++r) {
    auto idx = world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(0, 0, 0));
    world.robots[idx]->addSensor(
        std::make_unique<rb::GroundTruthPose>(1.0, 10));
    world.robots[idx]->addSensor(std::make_unique<rb::PositionalXYOdometry>(
        1.0, 10, std::normal_distribution<double>{0.0, 0.01}));
    world.dynamicsBodies.setPosition(
        idx, Eigen::Vector3d(r * 100.0, 0.0, 50.0));
  }
  world.addLandmark(Eigen::Vector3d(500, 500, 100));

  // Advance 3 seconds → GT records at t=0,1,2 = 3 samples per robot
  world.advanceWorld(3.0);

  pyfg::PfgWriterConfig config{};
  config.useGroundTruthOdometry = true;
  config.rangeVariance = 1.0;
  config.defaultPosePriorCov =
      pyfg::makeDiagUpperTri6x6(0.01, 0.01, 0.01, 0.04, 0.04, 0.04);
  config.landmarkPriorCovs.push_back(pyfg::makeDiagUpperTri3x3(0.1, 0.1, 0.1));
  config.odomRotationVariance = 0.04;

  // No range measurements for this test
  std::vector<sim::RangeMeasurement> emptyMeasurements;

  std::string testFile = "test_output.pfg";
  pyfg::writePfg(testFile, world, emptyMeasurements, config);

  auto lines = readLines(testFile);
  REQUIRE(!lines.empty());

  // Count lines by type
  size_t vertexPose = 0, vertexLandmark = 0, priorPose = 0, priorLandmark = 0;
  size_t edgeOdom = 0, edgeRange = 0;
  size_t lastVertexPose = 0, firstVertexLandmark = 0;
  size_t lastPrior = 0, firstEdge = 0;

  for (size_t i = 0; i < lines.size(); ++i) {
    const auto &l = lines[i];
    if (l.rfind("VERTEX_SE3:QUAT:PRIOR", 0) == 0) {
      priorPose++;
      lastPrior = i;
    } else if (l.rfind("VERTEX_SE3:QUAT", 0) == 0) {
      vertexPose++;
      lastVertexPose = i;
    } else if (l.rfind("VERTEX_XYZ:PRIOR", 0) == 0) {
      priorLandmark++;
      lastPrior = i;
    } else if (l.rfind("VERTEX_XYZ", 0) == 0) {
      vertexLandmark++;
      if (firstVertexLandmark == 0)
        firstVertexLandmark = i;
    } else if (l.rfind("EDGE_SE3:QUAT", 0) == 0) {
      edgeOdom++;
      if (firstEdge == 0)
        firstEdge = i;
    } else if (l.rfind("EDGE_RANGE", 0) == 0) {
      edgeRange++;
    }
  }

  // 2 robots × 3 GT samples = 6 pose vertices
  CHECK(vertexPose == 6);
  // 1 landmark
  CHECK(vertexLandmark == 1);
  // 2 initial pose priors (one per robot)
  CHECK(priorPose == 2);
  // 1 landmark prior
  CHECK(priorLandmark == 1);
  // 2 robots × 2 odom edges (3 poses → 2 edges each)
  CHECK(edgeOdom == 4);
  // No range measurements
  CHECK(edgeRange == 0);

  // Section ordering: vertices before landmarks before priors before edges
  CHECK(lastVertexPose < firstVertexLandmark);
  CHECK(firstVertexLandmark < lastPrior);  // landmarks before priors
  CHECK(lastPrior < firstEdge);

  // Check first line has correct field count (VERTEX_SE3:QUAT = 10 fields)
  std::istringstream firstLine(lines[0]);
  std::vector<std::string> fields;
  std::string field;
  while (firstLine >> field)
    fields.push_back(field);
  CHECK(fields.size() == 10);
  CHECK(fields[0] == "VERTEX_SE3:QUAT");
  // Pose name should be A0
  CHECK(fields[2] == "A0");

  // Clean up
  std::remove(testFile.c_str());
}

TEST_CASE("writePfg skips landmark priors when config is empty",
          "[pfgwriter]") {
  init_logger();

  rb::RbWorld world{};
  world.simData.dt = 1.0;
  world.createRngEngine(42);
  auto idx = world.addRobot<rb::ConstantVelRobot>(Eigen::Vector3d(0, 0, 0));
  world.robots[idx]->addSensor(
      std::make_unique<rb::GroundTruthPose>(1.0, 10));
  world.robots[idx]->addSensor(std::make_unique<rb::PositionalXYOdometry>(
      1.0, 10, std::normal_distribution<double>{0.0, 0.01}));
  world.addLandmark(Eigen::Vector3d(100, 0, 0));
  world.advanceWorld(1.0);

  pyfg::PfgWriterConfig config{};
  config.defaultPosePriorCov =
      pyfg::makeDiagUpperTri6x6(0.01, 0.01, 0.01, 0.04, 0.04, 0.04);
  // landmarkPriorCovs left empty → no landmark priors

  std::vector<sim::RangeMeasurement> empty;
  std::string testFile = "test_no_landmark_prior.pfg";
  pyfg::writePfg(testFile, world, empty, config);

  auto lines = readLines(testFile);
  for (const auto &l : lines) {
    CHECK(l.rfind("VERTEX_XYZ:PRIOR", 0) != 0);
  }

  std::remove(testFile.c_str());
}
