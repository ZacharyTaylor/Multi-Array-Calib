Header header

geometry_msgs/Vector3     pos_measured           # the measured body position
geometry_msgs/Vector3     pos_old                # the old body position
geometry_msgs/Vector3     vel_old                # the old body velocity
geometry_msgs/Vector3     pos_est                # the posteriori body position
geometry_msgs/Vector3     vel_est                # the posteriori body velocity

geometry_msgs/Quaternion  quat_measured          # the measured body orientation
geometry_msgs/Quaternion  quat_old               # the old body orientation
geometry_msgs/Vector3     omega_old              # the old body rate
geometry_msgs/Quaternion  quat_est               # the posteriori body orientation
geometry_msgs/Vector3     omega_est              # the posteriori body rate

std_msgs/Bool             outlier_flag           # flag indicating if the measurement at this timestep was detected as being an outlier
std_msgs/Bool             measurement_flip_flag  # flag indicating if the measurement from vicon has undergone a redundant flipping.

geometry_msgs/Quaternion  q_Z_Z1                 # the quaternion representing the rotation between subsequent measurements
geometry_msgs/Quaternion  q_Z_B                  # the quaternion representing the rotation between measurement and body
