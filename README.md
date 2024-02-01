# Robotics project

### Folder structure

- `planner/` contains a catkin package for kinematic calculations
- `vision_ros/` contains a catkin package that extracts blocks positions from the camera
- `vision_training/` contains the code that was used to train the YoLoS model that recognizes blocks

### Catkin workspace

The root folder of this repository is a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) created using `catkin_make`. In order for the workspace to be active, you need to run `catkin_make` in the root of this repository, and then `source devel/setup.bash`. The latter command can be put in `.bashrc` so that it's executed before starting any terminal session.

## Project n. 1
A number of objects (e.g., mega-blocks) are stored without any specific order on a stand
(initial stand) located within the workspace of a robotic manipulator. The manipulator is an
anthropomorphic arm, with a spherical wrist and a two-fingered gripper as end-effector.
The objects can belong to different classes but have a known geometry (coded in the STL files). The objective of the project is to use the manipulator to pick the objects in sequence and to position them on a different stand according to a specified order (final stand). A calibrated 3D sensor is used to locate the different objects and to detect their position in the initial stand. 
### Assignment
There are multiple objects on the initial stand, one for each class. There is no specific order
in the initial configuration, except that the base of the object is “naturally” in contact with the
ground. Each object has to be picked up and stored in the position prescribed for its class
and marked by the object’s silhouette.
### Delivery rules
The project is developed in groups. The typical group size consists of three-four members. We can also accept groups with a smaller number of members. The group is supposed to work in perfect cooperation and the workload is required to be fairly distributed. The specific contribution of each member will be exposed during the project discussion.
The delivery phase is as follows:

1. The project can be implemented both in simulation or on the real robot. In the second case it will have to be tested in the laboratory with the Teaching Assistant at most 5 least five days before the exam date. During the tests, small videos can be shot and used for the presentation.
2. Each group will have to deliver the package containing the full code (with doxygen documentation and a readme for use) plus a 5-6 pages report describing
    - the technique used for perception
    - the technique used for robot motion 
    - the technique used for high-level planning
3. The delivery deadline is three days before the (oral) exam presentation
4. On the day of the exam, the students will give a 10 minutes presentation highlighting the contribution of each member inside the oral session.
5. If allowed by the time, the group could also be asked to perform a small demo session. Otherwise, we will rely on the clip shot before the exam.
