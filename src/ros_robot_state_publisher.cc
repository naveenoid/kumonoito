#include "./../include/kumonoito/ros_robot_state_publisher.h"

#include <drake/multibody/rigid_body_tree.h>
#include <drake/systems/framework/basic_vector.h>
#include <rosgraph_msgs/Clock.h>

#include <string>

namespace kumonoito {

using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::State;
using drake::systems::kVectorValued;
using drake::systems::BasicVector;
using drake::systems::LeafSystem;

RosRobotStatePublisher::RosRobotStatePublisher(
    const RigidBodyTree<double> &tree,
    ros::NodeHandle *node_handle, int cache_length)
    : node_handle_(node_handle),
      joint_pub_(node_handle_->advertise<sensor_msgs::JointState>(
          "joint_states", cache_length)),
      clock_pub_(node_handle->advertise<rosgraph_msgs::Clock>("/clock",10)),
      tree_(tree){
  std::cout<<"statepublisher constructor 1\n";
  set_name("RosRobotStatePublisher");

  // Current version only supports fixed base systems since otherwise odom
  // trans messages must also be sent.
  DRAKE_DEMAND(tree_.get_num_velocities() == tree_.get_num_positions());

  std::cout<<"statepublisher constructor 2\n";
  const int vector_size = tree.get_num_positions()+ tree.get_num_velocities();
  DeclareInputPort(kVectorValued, vector_size);
  std::cout<<"statepublisher constructor 3\n";

  node_handle_->setParam("/use_sim_time", true);

  std::cout<<"statepublisher constructor 4\n";

  //this->DeclareDiscreteState(1);
  std::cout<<"statepublisher constructor 5\n";
}
//
//void RosRobotStatePublisher::DoCalcDiscreteVariableUpdates(
//    const Context<double> &context,
//    const std::vector<const DiscreteUpdateEvent<double> *> &,
//    DiscreteValues<double> *discrete_state) const {
//
//}

void RosRobotStatePublisher::set_publish_period(double period) {
LeafSystem<double>::DeclarePeriodicPublish(period);
}

void RosRobotStatePublisher::DoPublish(
    const Context<double> &context,
    const std::vector<const drake::systems::PublishEvent<double> *> &) const {
  // First publish clock time.
//  std::cout<<"start of do publish\n";
  rosgraph_msgs::Clock ros_time;
  ros_time.clock.fromSec(context.get_time());
  //  publish time to ros
  clock_pub_.publish(ros_time);

  // Obtains the input vector, which contains the generalized q,v state of the
  // RigidBodyTree.
  const BasicVector<double>* input_vector = EvalVectorInput(
      context, 0 /* port index */);

//  std::cout<<"middle of do publish\n";
  int num_positions = tree_.get_num_positions();
 drake::VectorX<double> x_positions = input_vector->CopyToVector();

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(num_positions);
  joint_state.position.resize(num_positions);

 for(int i = 0; i < num_positions; ++i) {
    joint_state.name[i] = tree_.get_position_name(i);
    joint_state.position[i] = x_positions(i);
  }
  //std::cout<<"gooing to ros publish do publish\n";

  joint_pub_.publish(joint_state);
 // std::cout<<"end of do publish\n";

}

void RosRobotStatePublisher::SetDefaultState(
    const Context<double> &context, State<double> *state) const {

}

} // namespace kumonoito