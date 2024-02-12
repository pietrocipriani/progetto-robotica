# KINEMATICS


# PLANNING

Operational space vs joint space

Use of quaternions

/// @note About the usage of quaternions:
/// - Euler angles have representational singularities and need the analytical jacobian.
/// - Angular velocities are much more intuitive that euler angles variations.
///   - Interpolation of euler angles can lead to "unexpected" movements.
///   - Quaternions consent numeric integration of the result (a problem of angular velocities).
/// However:
/// - Quaternions cannot be analytically integrated unless the rotation axis remains constant. // TODO: verify statement.
///   - Actually the project require rotations only around the Z axis. (loss of generality).
///   - Also in case of multiple rotations it is possible to segment the movement: cannot grant continuity in orientation variation.
/// - The project require rotations only around the Z axis (this is not a good point, loss of generality).
/// - The planner has full control on the orientation interpolation
