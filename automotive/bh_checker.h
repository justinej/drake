#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// BhChecker implements the Black Hole checker module that will
/// determine if the control is passed to the IDM or not. TODO Justine

/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: PoseVector for the ego car.
///   (InputPortDescriptor getter: controller_acceleration_input())
///
/// Input Port 1: FrameVelocity of the ego car.
///   (InputPortDescriptor getter: interlock_acceleration_input())
///
/// Input Port 2: PoseBundle for the acceleration cars, possibly inclusive of the ego
///   car's pose.
///   (InputPortDescriptor getter: acceleration_input())
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
class BhChecker : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BhChecker)

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
  BhChecker(const maliput::api::RoadGeometry& road,
                ScanStrategy path_or_branches,
                RoadPositionStrategy road_position_strategy,
                double period_sec);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit BhChecker(const BhChecker<U>& other)
      : BhChecker<T>(other.road_, other.path_or_branches_,
                         other.road_position_strategy_,
                         other.period_sec_) {}

  ~BhChecker() override;

  /// See the class description for details on the following input ports.
  /// @{
  // TODO Justine
  // for now,
  const systems::InputPortDescriptor<T>& controller_acceleration_input() const;
  const systems::InputPortDescriptor<T>& interlock_acceleration_input() const;
  const systems::InputPortDescriptor<T>& interlock_bh_bit_input() const;
  const systems::OutputPort<T>& acceleration_output() const;
  /// @}

 protected:
  const maliput::api::RoadGeometry& road() const { return road_; }
  int controller_acceleration_index() const { return controller_acceleration_index_; }
  int interlock_acceleration_index() const { return interlock_acceleration_index_; }
  int interlock_bh_bit_index() const { return interlock_bh_bit_index_; }
  int acceleration_output_index() const { return acceleration_output_index_; }

 private:

  template <typename> friend class BhChecker;

  // Converts @p pose into RoadPosition.

  const maliput::api::RoadGeometry& road_;
  const ScanStrategy path_or_branches_{};
  const RoadPositionStrategy road_position_strategy_{};
  const double period_sec_{};

  // Indices for the input / output ports.
  const int controller_acceleration_index_{};
  const int interlock_acceleration_index_{};
  const int interlock_bh_bit_index_{};
  const int acceleration_output_index_{};

  void CalcAcceleration(const systems::Context<T>& context,
                        systems::BasicVector<T>* accel_output) const;

};

}  // namespace automotive

namespace systems {
namespace scalar_conversion {
// Disable symbolic support, because we use ExtractDoubleOrThrow.
template <>
struct Traits<automotive::BhChecker> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake
