/// @file
///
/// This demo sets up a position controlled and gravity compensated kinova
/// jaco robot within a simulation, to reach and hold a given joint space pose.
/// The robot is initialized with an (arbitrary) joint space pose, and is
/// controlled to track and hold a final (arbitrary) joint space pose.

//#include <gflags/gflags.h>

#include <drake/common/find_resource.h>

#include "../include/kumonoito/jaco_common.h"

#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_tree_construction.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include "../include/kumonoito/ros_joint_state_publisher.h"

#include <ros/ros.h>

//DEFINE_double(simulation_sec, 2, "Number of seconds to simulate.");
const double FLAGS_simulation_sec = 2.0;
namespace kumonoito {
namespace kinova_jaco_arm {
namespace {

using drake::systems::DiagramBuilder;
using drake::systems::RigidBodyPlant;
using drake::systems::DrakeVisualizer;
using drake::VectorX;
using drake::systems::controllers::InverseDynamicsController;
using drake::systems::ConstantVectorSource;
using drake::systems::Diagram;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::VectorBase;

int DoMain(int argc, char **argv) {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  DiagramBuilder<double> builder;


  //Ros node handle stuff.
  ros::init(argc, argv, "jaco_sim");
  ros::NodeHandle node_handle;

  RigidBodyPlant<double>* plant = nullptr;

  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreeFromFixedModelAtPose(
        drake::FindResourceOrThrow(
            "drake/manipulation/models/jaco_description/urdf/j2n6s300.urdf"),
        tree.get());

    auto tree_sys =
        std::make_unique<RigidBodyPlant<double>>(std::move(tree));
    plant =
        builder.AddSystem<RigidBodyPlant<double>>(std::move(tree_sys));
    plant->set_name("plant");
  }

  // Creates and adds a RosJointStatePublisher for visualization.
  auto ros_joint_state_publisher = builder.AddSystem<RosJointStatePublisher>(
      plant->get_rigid_body_tree(), &node_handle, 1 /* cache length */,
      true /* enable_playback */);

  drake::lcm::DrakeLcm lcm;

  // Creates and adds LCM publisher for visualization.
  auto visualizer =
      builder.AddSystem<drake::systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm,
          true /* replay simulation */);


  // Adds a controller.
  VectorX<double> jaco_kp, jaco_kd, jaco_ki;
  SetPositionControlledJacoGains(&jaco_kp, &jaco_ki, &jaco_kd);
  auto control_sys =
      std::make_unique<InverseDynamicsController<double>>(
          plant->get_rigid_body_tree().Clone(), jaco_kp, jaco_ki, jaco_kd,
          false /* no feedforward acceleration */);
  auto controller =
      builder
          .AddSystem<InverseDynamicsController<double>>(
              std::move(control_sys));

  // Adds a constant source for desired state.
  Eigen::VectorXd const_pos = Eigen::VectorXd::Zero(kNumDofs * 2);
  const_pos(1) = 1.57;  // shoulder fore/aft angle, [rad]
  const_pos(2) = 2.0;   // elbow fore/aft angle, [rad]

  ConstantVectorSource<double>* const_src =
      builder.AddSystem<ConstantVectorSource<double>>(const_pos);

  const_src->set_name("constant_source");
  builder.Connect(const_src->get_output_port(),
                  controller->get_input_port_desired_state());

  // Connects the state port to the controller.
  static const int kInstanceId =
      RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;
  const auto& state_out_port =
      plant->model_instance_state_output_port(kInstanceId);
  builder.Connect(
      state_out_port, controller->get_input_port_estimated_state());

  // Connects the controller torque output to plant.
  const auto& torque_input_port =
      plant->model_instance_actuator_command_input_port(
          kInstanceId);
  builder.Connect(controller->get_output_port_control(),
                  torque_input_port);

  // Connects the ros_joint_state_publisher and builds the diagram.
  builder.Connect(
      plant->get_output_port(0),
      ros_joint_state_publisher->get_input_port(0));

  //Connects the drakeVisualizer
  builder.Connect(
      plant->get_output_port(0), visualizer->get_input_port(0));

  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  Simulator<double> simulator(*diagram);

  Context<double>& jaco_context = diagram->GetMutableSubsystemContext(
      *plant, &simulator.get_mutable_context());

  // Sets some (arbitrary) initial conditions.
  // See the @file docblock in jaco_common.h for joint index descriptions.
  VectorBase<double>& x0 =
      jaco_context.get_mutable_continuous_state_vector();
  x0.SetAtIndex(1, -1.57);  // shoulder fore/aft
  x0.SetAtIndex(2, -1.57);  // elbow fore/aft

  simulator.set_publish_at_initialization(false);

  simulator.Initialize();
  simulator.set_target_realtime_rate(1);

  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace kumonoito

int main(int argc, char* argv[]) {
  //gflags::ParseCommandLineFlags(&argc, &argv, true);
  return kumonoito::kinova_jaco_arm::DoMain(argc, argv);
}
