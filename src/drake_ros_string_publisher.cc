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
