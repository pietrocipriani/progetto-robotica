This part was done by Pietro Cipriani.

# KINEMATICS
## Direct Kinematics
Standard implementation for the direct kinematics with the usage of the Denavitt-Hartberg conventions.

Cartesian and cylindrical coordinates are supported for the position.\
Euler angles and quaternions are supported for the orientation.

## Inverse Kinematics
The inverse kinematics has been inspired by the course material.

The inverse kinematics produces "consistent" configurations:\
The idea is to prefer variations of $\theta_1$ for trasversal movements (parallel to the table).
Large movements of $\theta_2$ and $\theta_3$ have much more imprevedible effects in the operational space.\
For these purposes, $\theta_1$ is the first choosen angle.

No effort has been made in trying to reduce the joint movements (See planning for observations).

## Inverse Differential Kinematics
> Due to bad design choices, angular velocities have been represented with quaternions.
> The implementation is therefore buggy as quaternions represent "angles" in $4\pi$ modulus.
> Planning-level patches have been applied in order to successfully mitigate these problems.

_Inverse differential kinematics_ has been implemented with the **pseudo-inverse jacobian** approach.
> For this project a true inverse has been used due to the non-redundancy of the robot.

Euler angles are managed via a preemptive conversion to quaternions.

True **singularities** are not managed, with unexpected behaviours.

The implementation is generic (not specific for UR5), however secondary tasks have not been implemented
due to lack of interest (out of the scope of this project).

### Damped Least-squares Variant
A damped least-squares approach is available (requires recompilation).

The damping coefficient is dynamic and big just enough to grant a minimum threshold for the determinant.
For implementation details see the code.\
The determinant must be limited to avoid numeric approximation and also to "limit" the joint space movements.\
However, for the latter, the determinant threshold is just an approximation: an SVD approach should be preferred.
Also see the planning phase for joint space speed limitation.

The choice of a dynamic damping coefficient should be able to grant continuous velocity:
- The jacobians are composed of continue functions.
- The determinant of the jacobian is a continue function.
- The damping factor is a ReLU in function of the determinant: continuous.
- The DLS inverse is therefore continue.

However it produces very high accelerations when the threshold is very low.

This variant has been deactivated to avoid positioning errors.

### Feedback Variant
To avoid approximations due to the time quantization of the simulation, the linear approximation of the jacobian
and the integral drift of the configuration changes, a linear feedback is available.\
This variant adds to the desired movement, the movement necessary to move from the effective position to the desired one.


# PLANNING
The planning phase is designed as an hierarchy from low level primitives up to the top level planner.

## Generic Observations
The planner is capable of interpolating:
- In the joint space.
- In the operational space:
    - {Cylindrical, Cartesian} $\times$ {Euler Angles, Quaternions}
    - Some of the implementations have not been fully tested.
    - Cylindrical + Quaternions is the default configuration.

On the choice of cylindrical coordinates and quaternions:

Due to the presence of low-obstacles (other blocks), the planning requires a lifting phase, a trasversal movement and a dropping phase.\
To avoid the shoulder singularity it is advisable to perform radial movements.\
With this aim, cylindrical coordinates are preferred to cartesian coordinates.\
They also consent to decouple $\theta_1$ from $\theta_2,\theta_3,\theta_4$ and the **wrist** joints.\
Cylindrical coordinates consent to avoid the control panel in the back exploting the wrap-around of $\theta$.
> However with "bad" initial configurations, $\theta_1$ could reach the end of run.\
> It is also possible to exploit the numeric approximations to make the end effector warp the wrap-around.

Nb: only the interpolation is performed in cylindrical coordinates, the inverse kinematics uses cartesian coordinates
to avoid warps during the movement.

Quaternions are preferred to Euler angles as they do not present wrap-arounds and produce much more intuitive interpolations.
Euler Angles have also representational singularities, however this is not a problem with euler angles interpolation.\
A drawback of quaternions is that they are not easily analitically integrable (however are numerically integrable).


Drawbacks of joint space planning:

Interpolating in the joint space can lead to "unexpected" behaviours in the operational space.\
In the worst of the cases the arm can bump into the table.

## Movement Order Planner
> Note: unused

Plans which blocks move first in case some target destinations are already occupied.

## Top-level Planner
The top level planner returns a sequence of robot configurations in order to perform a smooth movement.

To avoid picking of blocks that cannot be positioned, the top level planner "elaborates" the full movement
for a specific block, with early exceptions in case of unreachability.

It produces a lazy sequence of movements, with negligible latencies.

Time dilation of the underlying operational space interpolation can occour to enforce joint speed limits.

The planner decides the via points for the parabolic interpolation:
- The starting and target pose are obviously present in the list of points.
- A via point is added just above one of the two points when the points are too low
    The lifting/dropping phase do not consent rotations to occour.
- A mid point can be added to guide the quaternion interpolation to avoid the end of run of $\theta_6$.
    Quaternions follow the shortest path.

To avoid the problems of the differential inverse kinematics, angular velocities have \[rad/sim\] as unit of measurement.

## Via-Points Planner
The via-points planner receives a list of via-points, produces a set of timestamps and returns a parabolic interpolation.\
Quaternions are implemented with a "stop&play" approach in order to avoid integration issues.
> This means that in the angular space each via-point is reached with null velocity. Velocity continuity is granted.\
> Quaternions product is non-abelian due to the presence of the cross product, integration is easier when the cross product is null.\
> The cross product is null with allineated axis of rotation or null rotations:
> - Allineated axis during the "linear" phase.
> - Null velocity during the "linear" phase joining.

Timestamps consider the length of the path, the maximum operational space speed and a "target" joint acceleration.

This estimation performs local optimization and do not produce optimal results globally.

To avoid the quaternion-velocity issues, the acceleration phase must last more than a second.

Only linear distance is considered, angular distance has not been added as care was needed to combine the two timings.

Joint space timing estimation should use a cylindrical path metric as a good approximation (actually the wrap around could bring to very odd results).

## Low-level Interpolation.
Standard implementations for the low level interpolation.

Prabolic interpolation implemented in order to actually respect the timings.

Choice of parabolic interpolation instead of cubic interpolation was quite arbitrary:
- Parabolic interpolation allows movement in the unsafe zone.
- Parabolic interpolation path is easier to measure.
