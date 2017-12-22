
/// @file
///
/// This demo sets up a simple passive dynamics simulation of the Kinova Jaco
/// arm. The robot is initialized with an (arbitrary) joint space pose, and is
/// simulated with zero torques at the joints.

//#include <gflags/gflags.h>

#include <drake/common/find_resource.h>
//#include <drake/common/text_logging_gflags.h>
#include "../include/kumonoito/jaco_common.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_tree_construction.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include "../include/kumonoito/ros_joint_state_publisher.h"

#include <ros/ros.h>

namespace kumonoito {

using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::ContinuousState;
using drake::systems::RigidBodyPlant;
using drake::systems::VectorBase;

namespace kinova_jaco_arm {
namespace {
//
//DEFINE_double(simulation_sec, 1.0, "Number of seconds to simulate");
//DEFINE_int32(replay_reps, -1, "No. of replays of simulation; Enter -1 for infinite replays");


const double FLAGS_simulation_sec = 1.0;
const int FLAGS_replay_reps = -1;

    int DoMain(int argc, char **argv) {
      DRAKE_DEMAND(FLAGS_simulation_sec > 0);

      double FLAGS_simulation_sec = 10000;
      drake::lcm::DrakeLcm lcm;
      drake::systems::DiagramBuilder<double> builder;

      //Ros node handle stuff.
      ros::init(argc, argv, "jaco_sim");
      ros::NodeHandle node_handle;

      std::cout<<"About to add plant\n";

      // Adds a plant.
      RigidBodyPlant<double> *plant = nullptr;
      {
        auto tree = std::make_unique<RigidBodyTree<double>>();
        drake::multibody::AddFlatTerrainToWorld(tree.get());
        kinova_jaco_arm::CreateTreeFromFixedModelAtPose(
                drake::FindResourceOrThrow(
                        "drake/manipulation/models/jaco_description/urdf/j2n6s300.urdf"),
                tree.get());

        auto tree_sys =
                std::make_unique<RigidBodyPlant<double>>(std::move(tree));
        plant = builder.AddSystem<RigidBodyPlant<double>>(
                std::move(tree_sys));
        plant->set_name("plant");
      }

      // Verifies the tree.
      const RigidBodyTree<double> &tree = plant->get_rigid_body_tree();

      // Creates and adds LCM publisher for visualization.
      auto visualizer =
              builder.AddSystem<drake::systems::DrakeVisualizer>(
                  tree, &lcm,
                  true /* replay simulation */);

      // Feeds in constant command inputs of zero.
      drake::VectorX<double> zero_values =
          drake::VectorX<double>::Zero(
          plant->get_input_size());
      auto zero_source =
              builder.AddSystem<
                  ConstantVectorSource<double>>(zero_values);
      zero_source->set_name("zero_source");
      builder.Connect(zero_source->get_output_port(),
                      plant->get_input_port(0));

      // Connects the visualizer and builds the diagram.
      builder.Connect(plant->get_output_port(0),
                      visualizer->get_input_port(0));

      // Sets up the ROSRobotStatePublisher
      auto ros_joint_state_publisher =
          builder.AddSystem<RosJointStatePublisher>(
              plant->get_rigid_body_tree(),
              &node_handle, 1 /* cache length */,
              true /* enable_playback */);

      builder.Connect(
          plant->get_output_port(0),
          ros_joint_state_publisher->get_input_port(0));

      std::unique_ptr<drake::systems::Diagram<double>>
              diagram = builder.Build();
      drake::systems::Simulator<double> simulator(*diagram);

      Context<double> &jaco_context =
          diagram->GetMutableSubsystemContext(
              *plant, &simulator.get_mutable_context());
      std::cout<<"Simulation  setting initial condition\n";
      // Sets (arbitrary) initial conditions.
      // See the @file docblock in jaco_common.h
      // for joint index descriptions.
      VectorBase<double>& x0 =
          jaco_context.get_mutable_continuous_state_vector();
      x0.SetAtIndex(1, 0.5);  // shoulder fore/aft angle [rad]

      simulator.set_publish_at_initialization(false);
     // ros_joint_state_publisher->set_publish_period(0.0);
      std::cout<<"Simulation about to initialize\n";
      simulator.Initialize();

      // Simulate for the desired duration.
      simulator.set_target_realtime_rate(1);
      simulator.StepTo(FLAGS_simulation_sec);

      std::cout<<"Simulation complete.";


      return 0;
    }

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace kumonoito


int main(int argc, char **argv) {
  return kumonoito::kinova_jaco_arm::DoMain(argc, argv);
}
