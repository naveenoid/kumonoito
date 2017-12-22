#pragma once

#include <drake/common/drake_copyable.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/common/eigen_types.h>
#include <drake/systems/primitives/signal_log.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

namespace kumonoito {

using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;

/**
 * This is a Drake System block that takes a RigidBodyTree and publishes
 * JointState messages that are intended for a RobotStatePublisher node which
 * when launched in concurrence with RViz renders the robot state.
 */
class RosJointStatePublisher : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RosJointStatePublisher)
  /**
   * Constructs the RosRobotStatePublisher.
   * @param tree - RigidBodyTree of the system that is to be rendered.
   * @param node - pointer to the NodeHandle.
   * @param cache_length - cached messages.
   */
  RosJointStatePublisher(
      const RigidBodyTree<double>& tree, ros::NodeHandle *node_handle,
      int cache_length, bool enable_playback = false);

  // Set the default to "initialization phase has not been completed."
  void SetDefaultState(const drake::systems::Context<double>&,
                       drake::systems::State<double>* state)
  const override;

  /**
   * Sets the publishing period of this system. See
   * LeafSystem::DeclarePublishPeriodSec() for details about the semantics of
   * parameter `period`.
   */
  void set_publish_period(double period);

  /**
   * Causes the visualizer to playback its cached data at real time.  If it has
   * not been configured to record/playback, a warning message will be written
   * to the log, but otherwise, no work will be done.
   */
  void ReplayCachedSimulation() const;

  /**
   * Plays back (at real time) a trajectory representing the input signal.
   */
  void PlaybackTrajectory(
      const PiecewisePolynomial<double>& input_trajectory) const;

 private:

  // Publishes a draw message if initialization is completed. Otherwise, it
  // emits a warning and return.
  void DoPublish(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::PublishEvent<double>*>&)
  const override;

  void PublishJointStateAndClock(drake::VectorX<double> x, double t) const;

  ros::NodeHandle* const node_handle_{};
  ros::Publisher joint_pub_{};
  ros::Publisher clock_pub_{};
  const RigidBodyTree<double>& tree_;

  std::unique_ptr<drake::systems::SignalLog<double>> log_{nullptr};
};


} // namespace kumonoito