#include "drake/automotive/bh_checker.h"

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
BhChecker<T>::BhChecker(const RoadGeometry& road,
                                ScanStrategy path_or_branches,
                                RoadPositionStrategy road_position_strategy,
                                double period_sec)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::BhChecker>{}),
      road_(road),
      path_or_branches_(path_or_branches),
      road_position_strategy_(road_position_strategy),
      period_sec_(period_sec),

      controller_acceleration_index_(
          this->DeclareInputPort(systems::kVectorValued, 1).get_index()),
      interlock_acceleration_index_(
          this->DeclareInputPort(systems::kVectorValued, 1).get_index()),
      interlock_bh_bit_index_(
          this->DeclareInputPort(systems::kVectorValued, 1).get_index()),
      acceleration_output_index_(this->DeclareVectorOutputPort(
        systems::BasicVector<T>(1),
        &BhChecker::CalcAcceleration).get_index())

      { /* intializing */ }

template <typename T>
BhChecker<T>::~BhChecker() {}


template <typename T>
const systems::InputPortDescriptor<T>& BhChecker<T>::controller_acceleration_input()
    const {
  return systems::System<T>::get_input_port(controller_acceleration_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& BhChecker<T>::interlock_acceleration_input()
    const {
  return systems::System<T>::get_input_port(interlock_acceleration_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& BhChecker<T>::interlock_bh_bit_input() const {
  return systems::System<T>::get_input_port(interlock_bh_bit_index_);
}

template <typename T>
const systems::OutputPort<T>& BhChecker<T>::acceleration_output() const {
  return systems::System<T>::get_output_port(acceleration_output_index_);
}

template <typename T>
void BhChecker<T>::CalcAcceleration(const systems::Context<T>& context,
                      systems::BasicVector<T>* accel_output) const {
    (*accel_output)[0] = 5.0; // TODO Justine
}

}  // namespace automotive
}  // namespace drake

// These instantiations must match the API documentation in idm_controller.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::automotive::BhChecker)
