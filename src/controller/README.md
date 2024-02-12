# The controller package

This part was done by Fabio Giovanazzi.

## What is it

The controller node, written in C++17, is the central component of the infrastructure that allows the robot to perform its block-moving tasks. It connects together all components:
1. the workspace is setup by spawning blocks in various places and creating target pads where the blocks are supposed to go
2. the `position_detection` package, written by Alessio Zeni, is used to find block poses in the workspace, which in turn uses the `block_detector` package to detect blocks
3. the `planner` package, written by Pietro Cipriani, is invoked to plan movements to pick up the various blocks and move them to their target pads
4. various ROS topics are used to interact with Gazebo and the UR5: spawning models, sending robot joint configs, reading initial joint configs, ...

## Model spawning

This package contains all of the `.sdf` models along with their meshes in `.stl` format, under the `models/` folder. There are two types of models:
- the `brick_*` models are those given in the project assignment, and fixed by Alessio Zeni (the STLs were slightly corrupted)
- the `pad_*` models were generated using Blender (see [pads.blend](./pads.blend)) and represent the target positions for blocks, i.e. the positions blocks are supposed to be moved to

The `.sdf` files were regenerated from scratch, so they all have a consistent format and use the alternative naming scheme defined in [vision_training/README.md#block-type-names](../../vision_training/README.md#block-type-names).

The C++ code that takes care of spawning blocks and pads:
- the `Spawner` class passes an runtime-generated SDF directly to the `/gazebo/spawn_sdf_model` Gazebo topic to allow for color and name customization
- there is also a `Deleter` to delete models, for easy workspace cleanup
- the code can be found under the `controller::world` workspace, and therefore at [include/controller/world](./include/controller/world/) and at [src/world](./src/world/)

## Control and planning

Actual planning is handled by Pietro Cipriani's `planner` library. The controller node wraps some of the `planner` functions and performs these operations:
- it takes care of the gripper position, so `prev_gripper_pos` is often passed around alongside `robot`
- it takes care of initializing the robot pose and the gripper position by reading data from the UR5's `/ur5/joint_states` topic
- it implements ways to wait for data about block positions to arrive from Alessio Zeni's `position_detection` publisher
- the code can be found under the `controller::control` workspace, and therefore at [include/controller/control](./include/controller/control/) and at [src/control](./src/control/)

## Experiments and command line options

While setting up the project we performed various experiments to check if everything was getting along nicely. Those can be found in the [include/controller/experiments.hpp](./include/controller/experiments.hpp) and [src/experiments.cpp](./src/experiments.cpp) files.

The "final" experiment (called `full`) is the one being executed by default, and which does all of the steps described at the beginning of this file, to abide by the project description goals.

It's still possible to run the older experiment by passing command line arguments when running the node:
```sh
# runs the default experiment, i.e. `full`
rosrun controller controller_node
# runs one of `full`, `all_blocks`, `selected_fixed_positions`,
#   or `workspace` experiments
rosrun controller controller_node [full|all_blocks|selected_fixed_positions|workspace]
```
