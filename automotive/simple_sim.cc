#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"

// A simple simulation of two cars in one lane, with parameters that can be
// accessed from the command line. The car in front is a Simple Car that travels
// at velocity {v1}, and immediately accelerates at acceleration {accel}. The car
// in back is a IDM/Interlock controlled Maliput Railcar that travels at velocity
// {v2}, distance {distance} behind the first car and attempts to brake in reaction
// to the car in front.


DEFINE_int32(v1, 50, "initial speed in mph of Simple Car in front");
DEFINE_int32(v2, 50, "initial speed in mph of IDM/Interlock-controlled Maliput Railcar in back");
DEFINE_double(accel, -10, "acceleration in m/s^2 of Simple Car in front starting at t=0");
DEFINE_double(distance, 50, "distance between the cars");



DEFINE_int32(num_simple_car, 1, "Number of SimpleCar vehicles. The cars are "
             "named \"0\", \"1\", \"2\", etc. If this option is provided, "
             "simple_car_names must not be provided.");
DEFINE_string(simple_car_names, "",
              "A comma-separated list that specifies the number of SimpleCar "
              "models to instantiate, their names, and the names of the LCM "
              "channels to which they subscribe (e.g., 'Russ,Jeremy,Liang' "
              "would spawn 3 cars subscribed to DRIVING_COMMAND_Russ, "
              "DRIVING_COMMAND_Jeremy, and DRIVING_COMMAND_Liang). If this "
              "option is provided, num_simple_car must not be provided.");
DEFINE_int32(num_mobil_car, 0,
             "Number of MOBIL-controlled SimpleCar vehicles. This option is "
             "currently only applied when the road network is a dragway. "
             "MOBIL-controlled vehicles are placed behind any idm-controlled "
             "railcars and any fixed-speed railcars.");
DEFINE_int32(num_idm_controlled_maliput_railcar, 1, "Number of IDM-controlled "
             "MaliputRailcar vehicles. This option is currently only applied "
             "when the road network is a dragway. These cars are added after "
             "the trajectory cars are added but before the fixed-speed "
             "railcars are added. They are initialized to be behind the "
             "fixed-speed railcars, if any.");
DEFINE_int32(num_maliput_railcar, 0, "Number of fixed-speed MaliputRailcar "
             "vehicles. This option is currently only applied when the road "
             "network is a dragway or merge. The speed is derived based on the "
             "road's base speed and speed delta. The railcars are added after "
             "the IDM-controlled railcars are added and are positioned in "
             "front of the IDM-controlled railcars.");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_sec, 10.0,
              "Number of seconds to simulate.");

DEFINE_int32(num_dragway_lanes, 1,
             "The number of lanes on the dragway. The number of lanes is by "
             "default zero to disable the dragway. A dragway road network is "
             "only enabled when the user specifies a number of lanes greater "
             "than zero. Only one road network can be enabled. Thus if this "
             "option is enabled, no other road network can be enabled.");
DEFINE_double(dragway_length, 400, "The length of the dragway.");
DEFINE_double(dragway_lane_width, 3.7, "The dragway lane width.");
DEFINE_double(dragway_shoulder_width, 3.0, "The dragway's shoulder width.");
DEFINE_double(dragway_base_speed, 4.0,
              "The speed of the vehicles on the right-most lane of the "
              "dragway.");
DEFINE_double(dragway_lane_speed_delta, 2,
              "The change in vehicle speed in the left-adjacent lane. For "
              "example, suppose the dragway has 3 lanes. Vehicles in the "
              "right-most lane will travel at dragway_base_speed m/s. "
              "Vehicles in the middle lane will travel at "
              "dragway_base_speed + dragway_lane_speed_delta m/s. Finally, "
              "vehicles in the left-most lane will travel at "
              "dragway_base_speed + 2 * dragway_lane_speed_delta m/s.");
DEFINE_double(dragway_vehicle_spacing, 10,
              "The initial spacing (in meters) between consecutive vehicles "
              "traveling on a lane.");

namespace drake {

using maliput::api::Lane;

namespace automotive {
namespace {

// The distance between the coordinates of consecutive rows of railcars and
// other controlled cars (e.g. MOBIL) on a dragway. 5 m ensures a gap between
// consecutive rows of Prius vehicles. It was empirically chosen.
constexpr double kRailcarRowSpacing{5};
constexpr double kControlledCarRowSpacing{5};

enum class RoadNetworkType {
  flat = 0,
  dragway = 1,
};

std::string MakeChannelName(const std::string& name) {
  std::string default_prefix{"DRIVING_COMMAND"};
  if (name.empty()) {
    return default_prefix;
  }
  return default_prefix + "_" + name;
}

// Adds a MaliputRailcar to the simulation involving a dragway. It throws a
// std::runtime_error if there is insufficient lane length for adding the
// vehicle.
//
// @param num_cars The number of vehicles to add.
//
// @param idm_controlled Whether the vehicle should be IDM-controlled.
//
// @param initial_s_offset The initial s-offset against which all vehicles are
// added. The vehicles are added in each lane of the dragway starting at this
// s-offset. Each row of vehicles is in front of the previous row (increasing
// s).
//
// @param dragway_road_geometry The road on which to add the railcars.
//
// @param simulator The simulator to modify.
void AddMaliputRailcar(int num_cars, bool idm_controlled, int initial_s_offset,
    const maliput::dragway::RoadGeometry* dragway_road_geometry,
    AutomotiveSimulator<double>* simulator) {
  for (int i = 0; i < num_cars; ++i) {
    const int lane_index = i % FLAGS_num_dragway_lanes;
    const double speed = FLAGS_v2;
    const MaliputRailcarParams<double> params;
    const Lane* lane =
        dragway_road_geometry->junction(0)->segment(0)->lane(lane_index);
    MaliputRailcarState<double> state;
    const int row = i / FLAGS_num_dragway_lanes;
    const double s_offset = initial_s_offset + kRailcarRowSpacing * row;
    if (s_offset >= lane->length()) {
      throw std::runtime_error(
          "Ran out of lane length to add a MaliputRailcar.");
    }
    state.set_s(s_offset);
    state.set_speed(speed);
    if (idm_controlled) {
      simulator->AddIdmControlledPriusMaliputRailcar(
          "IdmControlledMaliputRailcar" + std::to_string(i),
          LaneDirection(lane), ScanStrategy::kPath,
          RoadPositionStrategy::kExhaustiveSearch,
          0. /* time period (unused) */, params, state);
    } else {
      simulator->AddPriusMaliputRailcar("MaliputRailcar" + std::to_string(i),
                                        LaneDirection(lane), params, state);
    }
  }
}

// Adds SimpleCar instances to the simulator. It uses FLAGS_num_simple_car or
// FLAGS_simple_car_names to determine the number and names of SimpleCar
// instances to add. If both are specified, an exception will be thrown. The
// SimpleCar instances will start at X = 0 in the world frame, and will be
// offset along the world frame's Y-axis by a constant distance.
void AddSimpleCars(AutomotiveSimulator<double>* simulator) {
  const double kSimpleCarYSpacing{3};
  if (FLAGS_num_simple_car != 0 && !FLAGS_simple_car_names.empty()) {
    throw std::runtime_error("Both --num_simple_car and --simple_car_names "
        "specified. Only one can be specified at a time.");
  }
  if (FLAGS_num_simple_car != 0 || !FLAGS_simple_car_names.empty()) {
    std::string simple_car_names = FLAGS_simple_car_names;
    if (FLAGS_simple_car_names.empty()) {
      for (int i = 0; i < FLAGS_num_simple_car; ++i) {
        if (i != 0) {
          simple_car_names += ",";
        }
        simple_car_names += std::to_string(i);
      }
    }
    std::istringstream simple_car_name_stream(simple_car_names);
    std::string name;
    double y_offset{0};
    while (getline(simple_car_name_stream, name, ',')) {
      const std::string& channel_name = MakeChannelName(name);
      drake::log()->info("Adding simple car subscribed to {}.", channel_name);
      SimpleCarState<double> state;
      SimpleCarParams<double> params;
      state.set_y(y_offset);
      state.set_x(FLAGS_distance);
      state.set_velocity(FLAGS_v1);
      params.set_fixed_acceleration(FLAGS_accel);
      simulator->AddPriusSimpleCar(name, channel_name, state, params);
       y_offset += kSimpleCarYSpacing;
     }
  }
}

// Initializes the provided `simulator` with user-specified numbers of
// `SimpleCar` vehicles and `TrajectoryCar` vehicles. If parameter
// `road_network_type` equals `RoadNetworkType::dragway`, the provided
// `road_geometry` parameter must not be `nullptr`.
void AddVehicles(RoadNetworkType road_network_type,
    const maliput::api::RoadGeometry* road_geometry,
    AutomotiveSimulator<double>* simulator) {
  AddSimpleCars(simulator);

  if (road_network_type == RoadNetworkType::dragway) {
    DRAKE_DEMAND(road_geometry != nullptr);
    const maliput::dragway::RoadGeometry* dragway_road_geometry =
        dynamic_cast<const maliput::dragway::RoadGeometry*>(road_geometry);
    DRAKE_DEMAND(dragway_road_geometry != nullptr);
    for (int i = 0; i < FLAGS_num_mobil_car; ++i) {
      const int lane_index = i % FLAGS_num_dragway_lanes;
      const std::string name = "MOBIL" + std::to_string(i);
      SimpleCarState<double> state;
      const int row = i / FLAGS_num_dragway_lanes;
      const double x_offset = kControlledCarRowSpacing * row;
      const Lane* lane =
          dragway_road_geometry->junction(0)->segment(0)->lane(lane_index);
      if (x_offset >= lane->length()) {
        throw std::runtime_error(
            "Ran out of lane length to add new MOBIL-controlled SimpleCars.");
      }
      const double y_offset = lane->ToGeoPosition({0., 0., 0.}).y();
      state.set_x(x_offset);
      state.set_y(y_offset);
      simulator->AddMobilControlledSimpleCar(
          name, true /* with_s */, ScanStrategy::kPath,
          RoadPositionStrategy::kExhaustiveSearch,
          0. /* time period (unused) */, state);
    }

    AddMaliputRailcar(FLAGS_num_idm_controlled_maliput_railcar,
        true /* IDM controlled */, 0 /* initial s offset */,
        dragway_road_geometry, simulator);
    const double initial_s_offset =
        std::ceil(FLAGS_num_idm_controlled_maliput_railcar /
                  FLAGS_num_dragway_lanes) * kRailcarRowSpacing +
        std::ceil(FLAGS_num_mobil_car /
                  FLAGS_num_dragway_lanes) * kControlledCarRowSpacing;
    AddMaliputRailcar(FLAGS_num_maliput_railcar, false /* IDM controlled */,
        initial_s_offset, dragway_road_geometry, simulator);
  }
}

// Adds a flat terrain to the provided simulator.
void AddFlatTerrain(AutomotiveSimulator<double>*) {
  // Intentially do nothing. This is possible since only non-physics-based
  // vehicles are supported and they will not fall through the "ground" when no
  // flat terrain is present.
  //
  // TODO(liang.fok): Once physics-based vehicles are supported, a flat terrain
  // should actually be added. This can be done by adding method
  // `AutomotiveSimulator::AddFlatTerrain()`, which can then call
  // `drake::multibody::AddFlatTerrainToWorld()`. This method is defined in
  // drake/multibody/rigid_body_tree_construction.h.
}

// Adds a dragway to the provided `simulator`. The number of lanes, lane width,
// lane length, and the shoulder width are all user-specifiable via command line
// flags.
const maliput::api::RoadGeometry* AddDragway(
    AutomotiveSimulator<double>* simulator) {
  const double kMaximumHeight = 5.;  // meters
  const double kLinearTolerance = std::numeric_limits<double>::epsilon();
  const double kAngularTolerance = std::numeric_limits<double>::epsilon();
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry
      = std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId("Automotive Demo Dragway"),
          FLAGS_num_dragway_lanes,
          FLAGS_dragway_length,
          FLAGS_dragway_lane_width,
          FLAGS_dragway_shoulder_width,
          kMaximumHeight, kLinearTolerance, kAngularTolerance);
  return simulator->SetRoadGeometry(std::move(road_geometry));
}

// Adds a terrain to the simulated world. The type of terrain added depends on
// the provided `road_network_type` parameter. A pointer to the road network is
// returned. A return value of `nullptr` is possible if no road network is
// added.
const maliput::api::RoadGeometry* AddTerrain(RoadNetworkType road_network_type,
    AutomotiveSimulator<double>* simulator) {
  const maliput::api::RoadGeometry* road_geometry{nullptr};
  switch (road_network_type) {
    case RoadNetworkType::flat: {
      AddFlatTerrain(simulator);
      break;
    }
    case RoadNetworkType::dragway: {
      road_geometry = AddDragway(simulator);
      break;
    }
  }
  return road_geometry;
}

// Determines and returns the road network type based on the command line
// arguments.
RoadNetworkType DetermineRoadNetworkType() {
  int num_environments_selected{0};
  if (FLAGS_num_dragway_lanes) ++num_environments_selected;
  if (num_environments_selected > 1) {
    throw std::runtime_error("ERROR: More than one road network selected. Only "
        "one road network can be selected at a time.");
  }

  if (FLAGS_num_dragway_lanes > 0) {
    return RoadNetworkType::dragway;
  } else {
    return RoadNetworkType::flat;
  }
}

void printDistances(AutomotiveSimulator<double>* simulator) {
  const systems::rendering::PoseBundle<double> poses =
      simulator->GetCurrentPoses();
  const Isometry3<double>& pose_1 = poses.get_pose(0);
  const Isometry3<double>& pose_2 = poses.get_pose(1);
  drake::log()->info("Pose 1 translation is {}", pose_1.translation().x());
  drake::log()->info("Pose 2 translation is {}", pose_2.translation().x());
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  const RoadNetworkType road_network_type = DetermineRoadNetworkType();
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  auto* lcm = dynamic_cast<drake::lcm::DrakeLcm*>(simulator->get_lcm());
  DRAKE_DEMAND(lcm != nullptr);  // The simulator promises this should be OK.
  const maliput::api::RoadGeometry* road_geometry =
      AddTerrain(road_network_type, simulator.get());
  AddVehicles(road_network_type, road_geometry, simulator.get());
  lcm->StartReceiveThread();
  simulator->Start(FLAGS_target_realtime_rate);
  simulator->StepBy(FLAGS_simulation_sec);
  printDistances(simulator.get());
  simulator->StepBy(std::numeric_limits<double>::infinity()); // So simulation keeps running
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) { return drake::automotive::main(argc, argv); }
