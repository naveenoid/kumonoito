# Kumonoito
Kumonoito is the silken spidery thread that takes one from *Drake* to *ROS* and back. This thread, cast by the Great Warrior Gesar (**Russ Tedrake**) was woven in the hidden land of **Drake-Shambhala** where Buddhist values and traditions rule and can lead the hordes of the righteous but damned (**Drake Developers**) to climb up towards salvation (**ROS / LCM interoperability**). 

More practically, this repository exemplifies Drake simulations and systems that communicate via ROS and LCM messages.

## Organisation, Setup and Install
Currently, this repo is setup with 4 Drake Systems that enable ROS messaging communication : A _RosPublisherSystem_, a _RosSubscriberSystem_, a _RosJointStatePublisher_, and a _TfPublisherSystem_. 
It also includes a couple of simulation demos that utilize these systems in various ways and the associated models needed
to run these simulations.

Kumonoito is currently developed for and tested with ROS Kinetic and Ubuntu 16.04

### Install
Kumonoito depends on ROS and Drake.
1. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).
2. Install [Drake](http://drake.mit.edu/from_source.html) as a library. *Kumonoito* is currently tested with a locally built Drake library. 
To build Drake as a library,
    + Navigate to `$<drake-dir>`
    + Run `bazel run //:install -- $<drake_lib_dir>`
3. Clone *Kumonoito* : 
    + Navigate to `$<catkin_ws>/src/`
    + `git clone https://github.com/naveenoid/kumonoito.git`
4. Make your catkin workspace : `catkin_make`

### Demo Description / Setup
There are 2 demos available by default in *Kumonoito* and they are to some extent meant to be exemplary. These demos are built around the "Robotable" setup that consists of 2 Kinova Jaco Arms mounted to one side of a custom table. A few objects of interest for manipulation experiments are placed on the surface. The two projects are a (i) Passive "*Monolithic*" Sim of 2 Kinova Arms, and a (ii) Controlled *RoboTable* sim that responds to trajectory commands via ROS Messages [Note that controlled version message listening version is WIP - it currently is a monolithic style demo].
#### Demo Setup   
1. Install [Kinova ROS Package](https://github.com/Kinovarobotics/kinova-ros) (Note that only the kinova_description project is required at this stage).
2. Setup $DRAKE_RESOURCE_DIR : `export DRAKE_RESOURCE_ROOT=<drake_lib_dir>/drake/`

#### Demo Execution
1. Launch `roscore` from any terminal that has sourced the path from the appropriate ros build as well as catkin devel : 
    + First run `source /opt/ros/<ros-version>/setup.bash`
    + Then run `source <catkin_ws>/develop/setup.bash`
    + Now launch roscore (`roscore&`).
2. Launch the demos (recommended from a different terminal)
    + Source ros and catkin (First run `source /opt/ros/<ros-version/setup.bash`, then `source <catkin_ws>/develop/setup.bash`)
    + For the passive demo : `roslaunch <catkin_ws>/kumonoito/launch/passive_multi_jaco_sim.launch`
    + Once launched, if there are no problems, on the rviz window you should see the robot-table visualized on rviz and the 2 kinova Jaco arms *falling* passively. Various objects have been placed on the table.
    + For the controlled demo : `roslaunch <catkin_ws>/kumonoito/launch/controlled_robot_table_sim.launch`
    + if there are no problems, on the rviz window you should see the robot-table visualized on rviz and the 2 kinova Jaco arms move to an arbitrarily specified position in space. Various objects have been placed on the table.
    + Special note for both demos : Both demos simply publish to 1 kinova arm and the second arm uses a remap of the appropriate messages to render the same behaviour. This has been set for purely cosmetic reasons and a version with 2 independently controlled arms is WIP.
 
#### 
