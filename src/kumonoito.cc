#include <iostream>

#include <drake/common/drake_copyable.h> // devil
#include <drake/systems/framework/leaf_system.h>
#include "ros/ros.h" // deep sea
#include "std_msgs/String.h"

#include <cmath>

#include <drake/common/drake_assert.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/vector_system.h>

#include <cstdlib>
#include <iostream>
#include <ctime>


namespace shambhala {
namespace systems {

/**
 * Simple Continuous Time System
 *
 * xdot = -x + x^3
 * y = x
 */
class SimpleContinuousTimeSystem : public drake::systems::VectorSystem<double> {
 public:
  SimpleContinuousTimeSystem()
      : drake::systems::VectorSystem<double>(0,    // Zero inputs.
                                             1) {  // One output.
    this->DeclareContinuousState(1);               // One state variable.
  }

 private:
  // xdot = -x + x^3
  virtual void DoCalcVectorTimeDerivatives(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& input,
      const Eigen::VectorBlock<const Eigen::VectorXd>& state,
      Eigen::VectorBlock<Eigen::VectorXd>* derivatives) const {
    drake::unused(context, input);
    (*derivatives)(0) = -state(0) + std::pow(state(0), 3.0);
  }

  // y = x
  virtual void DoCalcVectorOutput(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& input,
      const Eigen::VectorBlock<const Eigen::VectorXd>& state,
      Eigen::VectorBlock<Eigen::VectorXd>* output) const {
    drake::unused(context, input);
    *output = state;
  }
};

}  // namespace systems
}  // namespace shambhala



namespace kumonoito {

int do_main(int argc, char **argv) {
  std::srand(std::time(0)); // use current time as seed for random generator
  std::cout<<"Hello Drake world. Drake successfully included.\n";

  // Create the simple system.
  shambhala::systems::SimpleContinuousTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<double> simulator(system);
//
//  // Set the initial conditions x(0).
//  drake::systems::ContinuousState<double>* state =
//      simulator.get_mutable_context()->get_mutable_continuous_state();

  drake::systems::VectorBase<double>* xc = simulator.get_mutable_context()->
      get_mutable_continuous_state_vector();
  xc->SetAtIndex(0, 1.0 * std::rand());
  // Simulate for 0.1 seconds.

  simulator.reset_integrator<drake::systems::RungeKutta3Integrator<double>>(system,
      simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);
  simulator.get_mutable_integrator()->set_requested_minimum_step_size(1e-3);
  simulator.get_mutable_integrator()->set_maximum_step_size(1e-3);
  simulator.Initialize();
  simulator.StepTo(0.1);

  double final_state =
  simulator.get_context().get_continuous_state_vector().GetAtIndex(0);

  std::cout<<"Final State :"<<final_state<<"\n";

  // Make sure the simulation converges to the stable fixed point at x=0.
  //DRAKE_DEMAND(*state[0] < 1.0e-4);

  // TODO(russt): Make a plot of the resulting trajectory.


  /**
 * The ros::init() function needs to see argc and argv so that it can perform
 * any ROS arguments and name remapping that were provided at the command line.
 * For programmatic remappings you can use a different version of init() which takes
 * remappings directly, but for most command-line programs, passing argc and argv is
 * the easiest way to do it.  The third argument to init() is the name of the node.
 *
 * You must call one of the versions of ros::init() before using any other
 * part of the ROS system.
 */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count<<", final state:"<<final_state;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }



  return 0;
}

} // namespace kumonoito

int main(int argc, char **argv) {
  return kumonoito::do_main(argc, argv);
}

/// @file
///
/// This demo sets up a simple passive dynamics simulation of the Kinova Jaco
/// arm. The robot is initialized with an (arbitrary) joint space pose, and is
/// simulated with zero torques at the joints.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kinova_jaco_arm/jaco_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::RigidBodyPlant;
using systems::VectorBase;

namespace examples {
namespace kinova_jaco_arm {
namespace {

DEFINE_double(simulation_sec, 2, "Number of seconds to simulate");

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreeFromFixedModelAtPose(
        FindResourceOrThrow(
            "drake/manipulation/models/jaco_description/urdf/j2n6s300.urdf"),
        tree.get());

    auto tree_sys =
        std::make_unique<RigidBodyPlant<double>>(std::move(tree));
    plant = builder.AddSystem<RigidBodyPlant<double>>(
        std::move(tree_sys));
    plant->set_name("plant");
  }

  // Verifies the tree.
  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  VerifyJacoTree(tree);

  // Creates and adds LCM publisher for visualization.
  auto visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  // Feeds in constant command inputs of zero.
  VectorX<double> zero_values = VectorX<double>::Zero(plant->get_input_size());
  auto zero_source =
      builder.AddSystem<ConstantVectorSource<double>>(zero_values);
  zero_source->set_name("zero_source");
  builder.Connect(zero_source->get_output_port(), plant->get_input_port(0));

  // Connects the visualizer and builds the diagram.
  builder.Connect(plant->get_output_port(0), visualizer->get_input_port(0));
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  Context<double>& jaco_context = diagram->GetMutableSubsystemContext(
      *plant, &simulator.get_mutable_context());

  // Sets (arbitrary) initial conditions.
  // See the @file docblock in jaco_common.h for joint index descriptions.
  VectorBase<double>& x0 = jaco_context.get_mutable_continuous_state_vector();
  x0.SetAtIndex(1, 0.5);  // shoulder fore/aft angle [rad]

  simulator.Initialize();

  // Simulate for the desired duration.
  simulator.set_target_realtime_rate(1);
  simulator.StepTo(FLAGS_simulation_sec);

  // Ensures the simulation was successful.
  const Context<double>& context = simulator.get_context();
  const ContinuousState<double>& state = context.get_continuous_state();
  const VectorBase<double>& position_vector = state.get_generalized_position();
  const VectorBase<double>& velocity_vector = state.get_generalized_velocity();

  const int num_q = position_vector.size();
  const int num_v = velocity_vector.size();

  // Ensures the sizes of the position and velocity vectors are correct.
  DRAKE_DEMAND(num_q == plant->get_num_positions());
  DRAKE_DEMAND(num_v == plant->get_num_velocities());
  DRAKE_DEMAND(num_q == num_v);

  // Ensures the robot's joints are within their position limits.
  const std::vector<std::unique_ptr<RigidBody<double>>>& bodies =
                                                           plant->get_rigid_body_tree().bodies;
  for (int state_index = 0, i = 0; i < static_cast<int>(bodies.size()); ++i) {
    // Skips rigid bodies without a parent. This includes the world.
    if (!bodies[i]->has_parent_body()) continue;

    const DrakeJoint& joint = bodies[i]->getJoint();
    const Eigen::VectorXd& min_limit = joint.getJointLimitMin();
    const Eigen::VectorXd& max_limit = joint.getJointLimitMax();

    // Defines a joint limit tolerance. This is the amount in radians over which
    // joint position limits can be violated and still be considered to be
    // within the limits. Once we are able to model joint limits via
    // constraints, we may be able to remove this tolerance value.
    const double kJointLimitTolerance = 0.0261799;  // 1.5 degrees.

    for (int j = 0; j < joint.get_num_positions(); ++j) {
      double position = position_vector.GetAtIndex(state_index++);
      if (position < min_limit[j] - kJointLimitTolerance) {
        std::cerr << "ERROR: Joint " + joint.get_name() + " (DOF " +
            joint.get_position_name(j) +
            ") violated minimum position limit (" +
            std::to_string(position) + " < " +
            std::to_string(min_limit[j]) + ").";
        return 1;
      }
      if (position > max_limit[j] + kJointLimitTolerance) {
        std::cerr << "ERROR: Joint " + joint.get_name() + " (DOF " +
            joint.get_position_name(j) +
            ") violated maximum position limit (" +
            std::to_string(position) + " > " +
            std::to_string(max_limit[j]) + ").";
        return 1;
      }
    }
  }

  return 0;
}
