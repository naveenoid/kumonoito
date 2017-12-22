
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
#include "../include/kumonoito/ros_publisher_system.h"

#include <ros/ros.h>

namespace kumonoito {
namespace kinova_jaco_arm {

namespace {
int DoMain(int argc, char **argv) {
  // Add a Constant Vector Source

  // Add a Publisher System


}

}
} // namespace

int main(int argc, char **argv) {
  return::kumonoito::kinova_jaco_arm::DoMain(argc, argv);

} // kinova_jaco_arm
} // kumonoito