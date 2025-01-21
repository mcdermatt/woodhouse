int32 scan_index                     # Index of the scan
int32 last_scan_index                # Index of the last keyframe (for relative odom constraint)
sensor_msgs/PointCloud2 point_cloud  # Point cloud of the keyframe
float32[] scan_context               # per scan context
float32[] ring_keys                  # per scan context
geometry_msgs/Pose odom_constraint   # Odometry Estimate
