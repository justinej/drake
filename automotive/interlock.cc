#include "drake/automotive/interlock.h"

#include <algorithm>
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

template <typename T>
Interlock<T>::Interlock(const RoadGeometry& road,
                                ScanStrategy path_or_branches,
                                RoadPositionStrategy road_position_strategy,
                                double period_sec)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::Interlock>{}),
      road_(road),
      path_or_branches_(path_or_branches),
      road_position_strategy_(road_position_strategy),
      period_sec_(period_sec),
      ego_pose_index_(
          this->DeclareVectorInputPort(PoseVector<T>()).get_index()),
      ego_velocity_index_(
          this->DeclareVectorInputPort(FrameVelocity<T>()).get_index()),
      traffic_index_(this->DeclareAbstractInputPort().get_index()),
      // num_poses_index_(this->DeclareAbstractInputPort().get_index()),
      pose_output_index_(
          this->DeclareVectorOutputPort(&Interlock::CalcPose).get_index()),
      velocity_output_index_(
          this->DeclareVectorOutputPort(&Interlock::CalcVelocity).get_index())
      // traffic_output_index_(
      //     this->DeclareAbstractOutputPort(&Interlock::MakePoseBundle,
      //                                     &Interlock::CalcTraffic).get_index())

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

// template <typename T>
// const systems::InputPortDescriptor<T>& Interlock<T>::num_poses_input() const {
//   return systems::System<T>::get_input_port(num_poses_index_);
// }

template <typename T>
const systems::OutputPort<T>& Interlock<T>::ego_pose_output() const {
  return systems::System<T>::get_output_port(pose_output_index_);
}

template <typename T>
const systems::OutputPort<T>& Interlock<T>::ego_velocity_output() const {
  return systems::System<T>::get_output_port(velocity_output_index_);
}

// template <typename T>
// const systems::OutputPort<T>& Interlock<T>::traffic_output() const {
//   return systems::System<T>::get_output_port(traffic_output_index_);
// }

template <typename T>
void Interlock<T>::CalcPose(
    const systems::Context<T>& context,
    PoseVector<T>* pose) const {
  const PoseVector<T>* const ego_pose_input =
      this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  pose->set_translation(ego_pose_input->get_translation());
  pose->set_rotation(ego_pose_input->get_rotation());
}

template <typename T>
void Interlock<T>::CalcVelocity(
    const systems::Context<T>& context,
    FrameVelocity<T>* velocity) const {
  const FrameVelocity<T>* const velocity_input =
      this->template EvalVectorInput<FrameVelocity>(context, ego_velocity_index_);
  velocity->set_velocity(velocity_input->get_velocity());
}

//template <typename T>
//PoseBundle<T> Interlock<T>::MakePoseBundle() const {
//    const int* const num_poses_input =
//      this->template EvalInputValue<int>(context, num_poses_index_);
//  return PoseBundle<T>(*num_poses_input);
//}
//
//template <typename T>
//void Interlock<T>::CalcTraffic(
//    const systems::Context<T>& context,
//    PoseBundle<T>* output) const {
//  const PoseBundle<T>* const traffic_input =
//      this->template EvalInputValue<PoseBundle<T>>(context, traffic_index_);
//  // *output = *traffic_input;
//  // we copy traffic because pose_bundle doesn't naturally have a copy-constructor
//  const int num_poses = traffic_input->get_num_poses();
//  for (int i = 0; i < num_poses; i++) {
//    // copy i-th pose from traffic_input to traffic
//    output->set_model_instance_id(i, traffic_input->get_model_instance_id(i));
//    output->set_name(i, traffic_input->get_name(i));
//    output->set_velocity(i, traffic_input->get_velocity(i));
//    output->set_pose(i, traffic_input->get_pose(i));
//  }
//}



}  // namespace automotive
}  // namespace drake

// These instantiations must match the API documentation in idm_controller.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::automotive::Interlock)
