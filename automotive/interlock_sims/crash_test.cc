// Adapted by Justine Jang from GTEST sample code by Vlad S.
// https://github.com/google/googletest/blob/master/googletest/samples/sample8_unittest.cc

// Runs tests on simple scenario with two cars in one lane.
// Car 1 (in front) travels at v1, brakes at time = 0 at a
// de-acceleration of `accel`. Distance to car 2 (in back) is
// `distance`. Test fails iff car 2 crashes into car 1 (as
// measured by distance along axes of the two cars and the lane).
//
// Params (v1, v2, accel, distance) that fail are then saved
// to a file (path should be modified to fit) which then can
// be visualized by running `python visualize_surface.py`

#include "drake/automotive/automotive_simulator.h"

#include "gtest/gtest.h"

#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/prius_vis.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/systems/rendering/pose_bundle.h"

#include <iostream>
#include <fstream>

namespace drake {

using maliput::api::Lane;

namespace automotive {
namespace {
#if GTEST_HAS_COMBINE

using ::testing::TestWithParam;
using ::testing::Bool;
using ::testing::ValuesIn;
using ::testing::Combine;

const std::vector<double> Range(const double start, const double end, const double step = 1) {
  std::vector<double> result;
  for (double i = start; i < end; i += step) {
    result.push_back(i);
  } return result;
}

// only one value (v1, v2, accel, distance) = (13, 16, -8, 2) as example
const std::vector<double> ALL_v1 = Range(13.0, 14.0, 1.0);      // car 1 velocities
const std::vector<double> ALL_v2 = Range(16.0, 17.0, 1.0);      // car 2 velocities
const std::vector<double> ALL_accel = Range(-8.0, -7.0, 1.0);   // acceleration of car 1 (front)
const std::vector<double> ALL_distance = Range(2, 3, 1.0);      // distance between car 1 and car 2

const int car_length = 5;

// TODO: change to absolute path of the file you want to write to
const std::string data_file_path = "/Users/justinej/Documents/drake/automotive/interlock_sims/data.txt";

class CrashTest : public TestWithParam< ::testing::tuple<double, double, double, double> > {
 protected:
  virtual void SetUp() {
    v1 = ::testing::get<0>(GetParam());
    v2 = ::testing::get<1>(GetParam());
    accel = ::testing::get<2>(GetParam());
    distance = ::testing::get<3>(GetParam());
  }
  virtual void TearDown() {}
  double v1;
  double v2;
  double accel;
  double distance;
};

int ClearFile() {
  std::ofstream myfile;
  myfile.open(data_file_path, std::ofstream::out | std::ofstream::trunc);
  myfile.close();
  return 0;
}


std::string SaveParams(const double v1, const double v2,
                        const double accel, const double distance) {
  std::ofstream myfile;
  myfile.open(data_file_path, std::ios::app);
  std::string params = std::to_string(v1) + "," + std::to_string(v2) +
                       "," + std::to_string(accel) + "," + std::to_string(distance);
  myfile << params + "\n";
  myfile.close();
  // drake::log()->info("Params are " + params);
  return ""; // "(" + params + ")";
}

const maliput::api::RoadGeometry* AddDragway(
    AutomotiveSimulator<double>* simulator) {
  const double kMaximumHeight = 5.;  // meters
  const double kLinearTolerance = std::numeric_limits<double>::epsilon();
  const double kAngularTolerance = std::numeric_limits<double>::epsilon();
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry
      = std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId("Automotive Demo Dragway"),
          1 /* num dragway lanes */,
          300 /* dragway length */,
          3.7 /*lane width */,
          3.0 /* shoulder length */,
          kMaximumHeight, kLinearTolerance, kAngularTolerance);
  return simulator->SetRoadGeometry(std::move(road_geometry));
}

int AddSimpleCar(AutomotiveSimulator<double>* simulator,
                  const double distance,
                  const double velocity,
                  const double acceleration) {
  SimpleCarState<double> initial_state_car1;
  initial_state_car1.set_x(distance);
  initial_state_car1.set_y(0);
  initial_state_car1.set_velocity(velocity);
  initial_state_car1.set_acceleration(acceleration);
  const int car1_id = simulator->AddPriusSimpleCar(
      "SimpleCar", "SimpleCarChannel", initial_state_car1);
  return car1_id;
}

int AddIdmControlledMaliputRailcar(AutomotiveSimulator<double>* simulator,
                                    const double velocity) {
  const maliput::api::RoadGeometry* road = AddDragway(simulator);
  const maliput::dragway::RoadGeometry* dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(road);
  const Lane* lane =
        dragway_road_geometry->junction(0)->segment(0)->lane(0);

  const MaliputRailcarParams<double> params;
  MaliputRailcarState<double> initial_state_car2;
  initial_state_car2.set_speed(velocity);

  const int car2_id = simulator->AddIdmControlledPriusMaliputRailcar(
      "IdmControlledMaliputCar",
      LaneDirection(lane),
      ScanStrategy::kPath,
      RoadPositionStrategy::kExhaustiveSearch,
      0. /* time period (unseen) */,
      params,
      initial_state_car2);
  return car2_id;
}

void CompareDistances(const systems::rendering::PoseBundle<double>& poses,
                      const int kCar1Index, const int kCar2Index,
                      const double v1, const double v2,
                      const double accel, const double distance) {
  const Isometry3<double>& pose_1 = poses.get_pose(kCar1Index);
  const Isometry3<double>& pose_2 = poses.get_pose(kCar2Index);
  // drake::log()->info("Pose 1 translation x is {}", pose_1.translation().x());
  // drake::log()->info("Pose 2 translation x is {}", pose_2.translation().x());
  EXPECT_LT(pose_2.translation().x() + car_length, pose_1.translation().x())
    << SaveParams(v1, v2, accel, distance);
}

TEST_P(CrashTest, ReadsSimpleCarState) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  const double kX_car1{distance};
  const double kVelocity_car1{v1};
  const double kVelocity_car2{v2};
  const double kAcceleration_car1{accel};
  const int kCar1Index{0};
  const int kCar2Index{1};

  const int car1_id = AddSimpleCar(simulator.get(), kX_car1, kVelocity_car1, kAcceleration_car1);
  const int car2_id = AddIdmControlledMaliputRailcar(simulator.get(), kVelocity_car2);

  simulator->Start(100.0); // speed up
  simulator->StepBy(10.0);

  const systems::rendering::PoseBundle<double> poses =
      simulator->GetCurrentPoses();
  ASSERT_EQ(poses.get_num_poses(), 2);
  ASSERT_EQ(poses.get_model_instance_id(kCar1Index), car1_id);
  ASSERT_EQ(poses.get_model_instance_id(kCar2Index), car2_id);
  
  CompareDistances(poses, kCar1Index, kCar2Index,
                   kVelocity_car1, kVelocity_car2, kAcceleration_car1, kX_car1);
}

// In order to run value-parameterized tests, you need to instantiate them,
// or bind them to a list of values which will be used as test parameters.
// You can instantiate them in a different translation module, or even
// instantiate them several times.
//
// Here, we instantiate our tests with a list of parameters. We must combine
// all variations of the boolean flag suppressing PrecalcPrimeTable and some
// meaningful values for tests. We choose a small value (1), and a value that
// will put some of the tested numbers beyond the capability of the
// PrecalcPrimeTable instance and some inside it (10). Combine will produce all
// possible combinations.

int cleared = ClearFile();
INSTANTIATE_TEST_CASE_P(MeaningfulTestParameters,
                        CrashTest,
                        Combine(ValuesIn(ALL_v1),
                                ValuesIn(ALL_v2),
                                ValuesIn(ALL_accel),
                                ValuesIn(ALL_distance)));

#else

// Google Test may not support Combine() with some compilers. If we
// use conditional compilation to compile out all code referring to
// the gtest_main library, MSVC linker will not link that library at
// all and consequently complain about missing entry point defined in
// that library (fatal error LNK1561: entry point must be
// defined). This dummy test keeps gtest_main linked in.
TEST(DummyTest, CombineIsNotSupportedOnThisPlatform) {}

#endif  // GTEST_HAS_COMBINE
}  // namespace
}  // namespace automotive
}  // namespace drake
