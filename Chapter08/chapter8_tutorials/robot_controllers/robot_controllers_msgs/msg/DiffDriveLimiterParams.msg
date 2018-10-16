# Various parameters for diff driver base velocity and acceleration limiter
# This message allows limits that would imposed by diff_drive_base
# to be understood by other components without needing access to 
# same set of rosparams, or URDF settings

float64 max_linear_velocity
float64 max_linear_acceleration

float64 max_angular_velocity
float64 max_angular_acceleration

# Wheel velocity limit are linear velocity (m/s) not angular velocities (rad/s)
float64 max_wheel_velocity

# distance between two wheels 
# used for calculating wheel velocities from angular velocity
float64 track_width

# If true limiter will reduce linear velocity
# when angular velocity is beyond limit so
# so path curvature is maintained
bool angular_velocity_limits_linear_velocity

# If true, linear and angular velocities will 
# scaled if wheel velocities beyond limits
# so path curvature is maintained
# otherwise wheel velocities are limited independently
bool scale_to_wheel_velocity_limits
