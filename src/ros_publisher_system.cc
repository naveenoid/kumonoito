#include "./../include/kumonoito/ros_publisher_system.h"

#include <memory>
#include <string>
#include <vector>

#include <drake/common/drake_copyable.h>
#include <drake/lcm/drake_lcm_interface.h>

#include <drake/systems/framework/leaf_system.h>

#include <ros/ros.h>

namespace kumonoito {

template <typename RosMessage>
RosPublisherSystem<RosMessage>::RosPublisherSystem(
    const std::string &topic, ros::NodeHandle *node_handle)
    : topic_(topic), node_handle_(node_handle) {
  DRAKE_DEMAND(node_handle_ != nullptr);

  publisher_ = node_handle->advertise<RosMessage>(topic, 0);

  DeclareAbstractInputPort();
  set_name(make_name(topic_));

}

template <typename RosMessage>
void RosPublisherSystem<RosMessage>::DoPublish(
    const drake::systems::Context<double> &context,
    const std::vector<
    const drake::systems::PublishEvent<double> *> &) const {
  SPDLOG_TRACE(drake::log(), "Publishing ROS {} message", topic_);

  const drake::systems::AbstractValue* const input_value =
      this->EvalAbstractInput(context, kPortIndex);
  DRAKE_ASSERT(input_value != nullptr);

  publisher_.publish(input_value->GetValue<RosMessage>());
}

}
