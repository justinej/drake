#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// Interlock implements the Black Hole checker module that will
/// determine if the control is passed to the IDM or not. TODO Justine

/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: PoseVector for the ego car.
///   (InputPortDescriptor getter: ego_pose_input())
///
/// Input Port 1: FrameVelocity of the ego car.
///   (InputPortDescriptor getter: ego_velocity_input())
///
/// Input Port 2: PoseBundle for the traffic cars, possibly inclusive of the ego
///   car's pose.
///   (InputPortDescriptor getter: traffic_input())
///
/// Output Port 0: A BasicVector containing the acceleration request.
///   (OutputPort getter: acceleration_output())
///
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_controllers
template <typename T>
class Interlock : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Interlock)

  /// Constructor.
  /// @param road The pre-defined RoadGeometry.
  /// @param path_or_branches If ScanStrategy::kBranches, performs IDM
  /// computations using vehicles detected in confluent branches; if
  /// ScanStrategy::kPath, limits to vehicles on the default path.  See
  /// documentation for PoseSelector::FindSingleClosestPose().
  /// @param road_position_strategy Determines whether or not to cache
  /// RoadPosition. See `calc_ongoing_road_position.h`.
  /// @param period_sec The update period to use if road_position_strategy ==
  /// RoadPositionStrategy::kCache.
  Interlock(const maliput::api::RoadGeometry& road,
            ScanStrategy path_or_branches,
            RoadPositionStrategy road_position_strategy,
            double period_sec,
            const double max_acceleration,
            const double max_deacceleration,
            const double max_speed,
            const double t_p,
            const double t_c,
            const double t_s
           );

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Interlock(const Interlock<U>& other)
      : Interlock<T>(other.road_, other.path_or_branches_,
                         other.road_position_strategy_,
                         other.period_sec_,
                         other.max_acceleration_,
                         other.max_deacceleration_,
                         other.max_speed_,
                         other.t_p_,
                         other.t_c_,
                         other.t_s_
                    ) {}

  ~Interlock() override;

  /// See the class description for details on the following input ports.
  /// @{
  // TODO Justine
  // for now,
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::InputPortDescriptor<T>& ego_velocity_input() const;
  const systems::InputPortDescriptor<T>& traffic_input() const;
  const systems::OutputPort<T>& acceleration_output() const;
  const systems::OutputPort<T>& bh_bit_output() const;
  /// @}

 protected:
  const maliput::api::RoadGeometry& road() const { return road_; }
  int ego_pose_index() const { return ego_pose_index_; }
  int ego_velocity_index() const { return ego_velocity_index_; }
  int traffic_index() const { return traffic_index_; }
  int acceleration_index() const { return acceleration_index_; }
  int bh_bit_index() const { return bh_bit_index_; }

 private:

  template <typename> friend class Interlock;

  // Converts @p pose into RoadPosition.

  const maliput::api::RoadGeometry& road_;
  const ScanStrategy path_or_branches_{};
  const RoadPositionStrategy road_position_strategy_{};
  const double period_sec_{};
  const double max_acceleration_{};
  const double max_deacceleration_{};
  const double max_speed_{};
  const double t_p_{};
  const double t_c_{};
  const double t_s_{};

  // Indices for the input / output ports.
  const int ego_pose_index_{};
  const int ego_velocity_index_{};
  const int traffic_index_{};
  const int acceleration_index_{};
  const int bh_bit_index_{};
  // const int traffic_output_index_{};

  void CalcAcceleration(const systems::Context<T>& context,
                        systems::BasicVector<T>* accel_output) const;
  void CalcBhBit(const systems::Context<T>& context,
                        systems::BasicVector<T>* bh_bit_output) const;
  void ImplCalcBhBit(
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::FrameVelocity<T>& ego_velocity,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const maliput::api::RoadPosition& ego_rp,
      systems::BasicVector<T>* bh_output) const;

};

}  // namespace automotive

namespace systems {
namespace scalar_conversion {
// Disable symbolic support, because we use ExtractDoubleOrThrow.
template <>
struct Traits<automotive::Interlock> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake
