#include "../include/kumonoito/ros_subscriber_system.h"

#include <drake/systems/framework/leaf_context.h>
#include <drake/systems/framework/leaf_system.h>

#include <ros/ros.h>

namespace kumonoito {
template <typename RosMessage>
RosSubscriberSystem<RosMessage>::RosSubscriberSystem(
    const std::string &topic, ros::NodeHandle *node_handle) :
    topic_(topic), node_handle_(node_handle) {
  DRAKE_DEMAND(node_handle_);

  subscriber_ = node_handle->subscribe(
      topic, 100, &RosSubscriberSystem<RosMessage>::HandleMessage, this);

  DeclareAbstractOutputPort(
      [this](const drake::systems::Context<double>&) {
        return this->AllocateOutputValue();
      },
      [this](const drake::systems::Context<double>& context,
             drake::systems::AbstractValue* out) {
        this->CalcOutputValue(context, out);
      });

  set_name(make_name(topic_));
}

template <typename RosMessage>
void RosSubscriberSystem<RosMessage>::DoCalcNextUpdateTime(
    const drake::systems::Context<double> &context,
    drake::systems::CompositeEventCollection<double> *events,
    double *time) const {
  const int last_message_count = GetMessageCount(context);

  const int received_message_count = [this]() {
    std::unique_lock<std::mutex> lock(received_message_mutex_);
    return received_message_count_;
  }();

  // Has a new message. Schedule an update event.
  if (last_message_count != received_message_count) {
    // TODO(siyuan): should be context.get_time() once #5725 is resolved.
    *time = context.get_time() + 0.0001;

    drake::systems::EventCollection<drake::systems::UnrestrictedUpdateEvent<double>>&
        uu_events = events->get_mutable_unrestricted_update_events();
    uu_events.add_event(
        std::make_unique<drake::systems::UnrestrictedUpdateEvent<double>>(
            drake::systems::Event<double>::TriggerType::kTimed));
  }
}

template <typename RosMessage>
void RosSubscriberSystem<RosMessage>::DoCalcUnrestrictedUpdate(
    const drake::systems::Context<double> &,
    const std::vector<
        const drake::systems::UnrestrictedUpdateEvent<
            double> *> &,
    drake::systems::State<double> *state) const {
  ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
}

template <typename RosMessage>
std::unique_ptr<drake::systems::AbstractValues>
RosSubscriberSystem<RosMessage>::AllocateAbstractState() const  {
  std::vector<std::unique_ptr<drake::systems::AbstractValue>> abstract_vals(2);
  abstract_vals[kStateIndexMessage] =
      this->RosSubscriberSystem::AllocateOutputValue();
  abstract_vals[kStateIndexMessageCount] =
      drake::systems::AbstractValue::Make<int>(0);
  return std::make_unique<drake::systems::AbstractValues>(std::move(abstract_vals));
}

template <typename RosMessage>
void RosSubscriberSystem<RosMessage>::ProcessMessageAndStoreToAbstractState(
    drake::systems::AbstractValues *abstract_state) const {
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  abstract_state->get_mutable_value(kStateIndexMessage)
      .GetMutableValue<RosMessage>() = received_message_;
  abstract_state->get_mutable_value(kStateIndexMessageCount)
      .GetMutableValue<int>() = received_message_count_;
};

template <typename RosMessage>
void RosSubscriberSystem<RosMessage>::HandleMessage(const RosMessage &message)  {
  SPDLOG_TRACE(drake::log(), "Receiving ROS {} message", topic_);
  std::lock_guard<std::mutex> lock(received_message_mutex_);
  received_message_ = message;
  received_message_count_++;
  received_message_condition_variable_.notify_all();
}





} // namespace kumonoito
