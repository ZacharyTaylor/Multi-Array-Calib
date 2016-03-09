package ros_vrpn_client;

public interface viconEstimator extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ros_vrpn_client/viconEstimator";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\ngeometry_msgs/Vector3     pos_measured           # the measured body position\r\ngeometry_msgs/Vector3     pos_old                # the old body position\r\ngeometry_msgs/Vector3     vel_old                # the old body velocity\r\ngeometry_msgs/Vector3     pos_est                # the posteriori body position\r\ngeometry_msgs/Vector3     vel_est                # the posteriori body velocity\r\n\r\ngeometry_msgs/Quaternion  quat_measured          # the measured body orientation\r\ngeometry_msgs/Quaternion  quat_old               # the old body orientation\r\ngeometry_msgs/Vector3     omega_old              # the old body rate\r\ngeometry_msgs/Quaternion  quat_est               # the posteriori body orientation\r\ngeometry_msgs/Vector3     omega_est              # the posteriori body rate\r\n\r\nstd_msgs/Bool             outlier_flag           # flag indicating if the measurement at this timestep was detected as being an outlier\r\nstd_msgs/Bool             measurement_flip_flag  # flag indicating if the measurement from vicon has undergone a redundant flipping.\r\n\r\ngeometry_msgs/Quaternion  q_Z_Z1                 # the quaternion representing the rotation between subsequent measurements\r\ngeometry_msgs/Quaternion  q_Z_B                  # the quaternion representing the rotation between measurement and body\r\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getPosMeasured();
  void setPosMeasured(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getPosOld();
  void setPosOld(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getVelOld();
  void setVelOld(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getPosEst();
  void setPosEst(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getVelEst();
  void setVelEst(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getQuatMeasured();
  void setQuatMeasured(geometry_msgs.Quaternion value);
  geometry_msgs.Quaternion getQuatOld();
  void setQuatOld(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getOmegaOld();
  void setOmegaOld(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getQuatEst();
  void setQuatEst(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getOmegaEst();
  void setOmegaEst(geometry_msgs.Vector3 value);
  std_msgs.Bool getOutlierFlag();
  void setOutlierFlag(std_msgs.Bool value);
  std_msgs.Bool getMeasurementFlipFlag();
  void setMeasurementFlipFlag(std_msgs.Bool value);
  geometry_msgs.Quaternion getQZZ1();
  void setQZZ1(geometry_msgs.Quaternion value);
  geometry_msgs.Quaternion getQZB();
  void setQZB(geometry_msgs.Quaternion value);
}
