#include "../include/kumonoito/ros_joint_state_publisher.h"

#include <drake/multibody/rigid_body_tree.h>
#include <drake/systems/framework/basic_vector.h>
#include <rosgraph_msgs/Clock.h>
#include <drake/systems/primitives/signal_log.h>
#include <drake/common/eigen_types.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include <string>

namespace kumonoito {

using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::State;
using drake::systems::kVectorValued;
using drake::systems::BasicVector;
using drake::systems::LeafSystem;
using drake::systems::SignalLog;

RosJointStatePublisher::RosJointStatePublisher(
    const RigidBodyTree<double> &tree,
    ros::NodeHandle *node_handle, int cache_length, bool enable_playback)
    : node_handle_(node_handle),
      joint_pub_(node_handle_->advertise<sensor_msgs::JointState>(
          "joint_states", cache_length)),
      clock_pub_(node_handle->advertise<rosgraph_msgs::Clock>("/clock",10)),
      tree_(tree){
  set_name("RosJointStatePublisher");

  // Current version only supports fixed base systems since otherwise odom
  // trans messages must also be sent.
  DRAKE_DEMAND(tree_.get_num_velocities() == tree_.get_num_positions());

  const int vector_size = tree.get_num_positions()+ tree.get_num_velocities();
  DeclareInputPort(kVectorValued, vector_size);

  node_handle_->setParam("/use_sim_time", true);

  if (enable_playback) log_.reset(new SignalLog<double>(vector_size));
}

void RosJointStatePublisher::set_publish_period(double period) {
LeafSystem<double>::DeclarePeriodicPublish(period);
}

void RosJointStatePublisher::DoPublish(
    const Context<double> &context,
    const std::vector<const drake::systems::PublishEvent<double> *> &) const {
  PublishJointStateAndClock(
      EvalVectorInput(context, 0 /* port index */)->CopyToVector(),
      context.get_time());
}

void RosJointStatePublisher::PublishJointStateAndClock(
    drake::VectorX<double> x, double t) const {
  // First publish clock time.
  rosgraph_msgs::Clock ros_time;
  ros_time.clock.fromSec(t);

  //  publish time to ros
  clock_pub_.publish(ros_time);

  int num_positions = tree_.get_num_positions();

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(num_positions);
  joint_state.position.resize(num_positions);

  for(int i = 0; i < num_positions; ++i) {
    joint_state.name[i] = tree_.get_position_name(i);
    joint_state.position[i] = x(i);
  }
  joint_pub_.publish(joint_state);
}

void RosJointStatePublisher::SetDefaultState(
    const Context<double> &context, State<double> *state) const {

}

void RosJointStatePublisher::ReplayCachedSimulation() const {
    if (log_ != nullptr) {
      // Build piecewise polynomial
      auto times = log_->sample_times();
      // NOTE: The SignalLog can record signal for multiple identical time stamps.
      //  This culls the duplicates as required by the PiecewisePolynomial.
      std::vector<int> included_times;
      included_times.reserve(times.rows());
      std::vector<double> breaks;
      included_times.push_back(0);
      breaks.push_back(times(0));
      int last = 0;
      for (int i = 1; i < times.rows(); ++i) {
        double val = times(i);
        if (val != breaks[last]) {
          breaks.push_back(val);
          included_times.push_back(i);
          ++last;
        }
      }

      auto sample_data = log_->data();
      std::vector<drake::MatrixX<double>> knots;
      knots.reserve(sample_data.cols());
      for (int c : included_times) {
        knots.push_back(sample_data.col(c));
      }
      auto func = PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots);

      PlaybackTrajectory(func);
    } else {
      drake::log()->warn(
          "DrakeVisualizer::ReplayCachedSimulation() called on instance that "
              "wasn't initialized to record. Next time, please construct "
              "DrakeVisualizer with recording enabled.");
    }
}

void RosJointStatePublisher::PlaybackTrajectory(
    const PiecewisePolynomial<double> &input_trajectory) const {
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  // Target frame length at 60 Hz playback rate.
  const double kFrameLength = 1 / 60.0;
  double sim_time = input_trajectory.getStartTime();
  TimePoint prev_time = Clock::now();
  BasicVector<double> data(log_->get_input_size());
  while (sim_time < input_trajectory.getEndTime()) {
    data.set_value(input_trajectory.value(sim_time));
    PublishJointStateAndClock(data.CopyToVector(), sim_time);

    const TimePoint earliest_next_frame = prev_time + Duration(kFrameLength);
    std::this_thread::sleep_until(earliest_next_frame);
    TimePoint curr_time = Clock::now();
    sim_time += (curr_time - prev_time).count();
    prev_time = curr_time;
  }

  // Final evaluation is at the final time stamp, guaranteeing the final state
  // is visualized.
  data.set_value(input_trajectory.value(input_trajectory.getEndTime()));
  PublishJointStateAndClock(data.CopyToVector(), input_trajectory.getEndTime());
}

} // namespace kumonoito