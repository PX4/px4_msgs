# Trajectory setpoint in NED frame
# Input to the fully actuated Voliro position and attitude controller.

uint64 timestamp # time since system start (microseconds)

# NED local world frame
float32[3] position               # in meters
float32[3] velocity               # in meters/second
float32[3] acceleration           # in meters/second^2
float32[3] jerk                   # in meters/second^3 (for logging only)

# Body FRD attitude and motion
float32[4] quaternion             # unit quaternion, w x y z, body FRD to NED
float32[3] angular_velocity       # desired body angular velocity in radians/second
