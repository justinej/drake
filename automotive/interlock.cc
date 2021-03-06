#include "drake/automotive/interlock.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::GeoPositionT;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::LanePositionT;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

// A very large distance to make sure we can always detect a potential danger ahead.
const double kScanAheadDistance = 300.0;

template <typename T>
Interlock<T>::Interlock(const RoadGeometry& road,
                        ScanStrategy path_or_branches,
                        RoadPositionStrategy road_position_strategy,
                        double period_sec,
                        const double max_acceleration,
                        const double max_deacceleration,
                        const double max_speed,
                        const double t_p,
                        const double t_c,
                        const double t_s
                       )
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::Interlock>{}),
      road_(road),
      path_or_branches_(path_or_branches),
      road_position_strategy_(road_position_strategy),
      period_sec_(period_sec),
      max_acceleration_(max_acceleration),
      max_deacceleration_(max_deacceleration),
      max_speed_(max_speed),
      t_p_(t_p),
      t_c_(t_c),
      t_s_(t_s),
      ego_pose_index_(
          this->DeclareVectorInputPort(PoseVector<T>()).get_index()),
      ego_velocity_index_(
          this->DeclareVectorInputPort(FrameVelocity<T>()).get_index()),
      traffic_index_(this->DeclareAbstractInputPort().get_index()),
      acceleration_index_(this->DeclareVectorOutputPort(
        systems::BasicVector<T>(1),
        &Interlock::CalcAcceleration).get_index()),
      bh_bit_index_(this->DeclareVectorOutputPort(
        systems::BasicVector<T>(1),
        &Interlock::CalcBhBit).get_index())

      { /* intializing */ }

template <typename T>
Interlock<T>::~Interlock() {}


template <typename T>
const systems::InputPortDescriptor<T>& Interlock<T>::ego_pose_input()
    const {
  return systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& Interlock<T>::ego_velocity_input()
    const {
  return systems::System<T>::get_input_port(ego_velocity_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& Interlock<T>::traffic_input() const {
  return systems::System<T>::get_input_port(traffic_index_);
}

template <typename T>
const systems::OutputPort<T>& Interlock<T>::acceleration_output() const {
  return systems::System<T>::get_output_port(acceleration_index_);
}

template <typename T>
const systems::OutputPort<T>& Interlock<T>::bh_bit_output() const {
  return systems::System<T>::get_output_port(bh_bit_index_);
}

template <typename T>
void Interlock<T>::CalcAcceleration(const systems::Context<T>& context,
                      systems::BasicVector<T>* accel_output) const {
    (*accel_output)[0] = -max_deacceleration_;
}

// Returns 1 to trigger Interlock, 0 to not trigger
template <typename T>
void Interlock<T>::CalcBhBit(const systems::Context<T>& context,
                      systems::BasicVector<T>* bh_bit_output) const {
  // TODO: A separate files to store parameters?

  // Obtain the necessary input data for determing whether we are
  // close to entering "event horizon".
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context,
                                                    ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  const PoseBundle<T>* const traffic_poses =
      this->template EvalInputValue<PoseBundle<T>>(context, traffic_index_);
  DRAKE_ASSERT(traffic_poses != nullptr);

  // Obtain the state if we've allocated it.
  RoadPosition ego_rp;
  if (context.template get_state().get_abstract_state().size() != 0) {
    DRAKE_ASSERT(context.get_num_abstract_states() == 1);
    ego_rp = context.template get_abstract_state<RoadPosition>(0);
  }
  return ImplCalcBhBit(*ego_pose, *ego_velocity, *traffic_poses, ego_rp,
                       bh_bit_output);
}

template <typename T>
void Interlock<T>::ImplCalcBhBit(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const RoadPosition& ego_rp,
    systems::BasicVector<T>* bh_output) const {
  using std::abs;
  using std::max;

  RoadPosition ego_position = ego_rp;
  if (!ego_rp.lane) {
    const auto gp =
        GeoPositionT<T>::FromXyz(ego_pose.get_isometry().translation());
    ego_position =
        road_.ToRoadPosition(gp.MakeDouble(), nullptr, nullptr, nullptr);
  }

  // Find the single closest car ahead.
  const ClosestPose<T> lead_car_pose = PoseSelector<T>::FindSingleClosestPose(
      ego_position.lane, ego_pose, traffic_poses,
      kScanAheadDistance, AheadOrBehind::kAhead,
      path_or_branches_);
  const T headway_distance = lead_car_pose.distance;

  const LanePositionT<T> lane_position(T(ego_position.pos.s()),
                                       T(ego_position.pos.r()),
                                       T(ego_position.pos.h()));
  const T s_dot_ego = PoseSelector<T>::GetSigmaVelocity(
      {ego_position.lane, lane_position, ego_velocity});

  // const T s_dot_lead =
  //     (abs(lead_car_pose.odometry.pos.s()) ==
  //      std::numeric_limits<T>::infinity())
  //         ? T(0.)
  //         : PoseSelector<T>::GetSigmaVelocity(lead_car_pose.odometry);

  // Calculate velocity of the ego car relative to the closest car ahead.
  // const T closing_velocity = s_dot_ego - s_dot_lead;

  const T max_accel = T(max_acceleration_);
  const T t_p = T(t_p_);
  const T t_c = T(t_c_);
  const T t_s = T(t_s_);
  const T total_delay = t_p + t_s + t_c;
  const T minimum_braking_dist = s_dot_ego + max_accel * t_p + max_accel*(t_p + t_s + t_c);
  // See slides for derivation
  const T dist_stop = max_accel * (t_p*t_p + total_delay*total_delay )/2 +
                      (s_dot_ego + max_accel * t_p) * (2*t_p + t_s + t_c) +
                      minimum_braking_dist*minimum_braking_dist / (2*max_deacceleration_);

  // Compute the output from the interlock function: 1 to trigger Interlock, 0 if safe
  if (headway_distance - T(5.0 /* length of car */) < dist_stop) {
    (*bh_output)[0] = 1;
  } else {
    (*bh_output)[0] = 0;
  }
}

}  // namespace automotive
}  // namespace drake

// These instantiations must match the API documentation in idm_controller.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::automotive::Interlock)
