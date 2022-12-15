# Header
std_msgs/Header header 

# Initial guess from odom
float32 initial_guess_x
float32 initial_guess_y
float32 initial_guess_z
float32 initial_guess_rot_w
float32 initial_guess_rot_x
float32 initial_guess_rot_y
float32 initial_guess_rot_z

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
sensor_msgs/PointCloud2 cloud_edge      # extracted edge features
sensor_msgs/PointCloud2 cloud_flat      # extracted flat features