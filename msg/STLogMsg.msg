# Message that contains logs for Stop motion cue
# Header seq and frame_id are retrieved from Body Image Frame from Astra SDK, timestamp is the current time
std_msgs/Header header

# Technical Stop parameter, id, duration in seconds, velocity in m/s, coordinates where robot stops
std_msgs/String id
float64 total_duration
float64 wait_seconds

geometry_msgs/Point32 robot_coord

float64 robot_lin_vel_x

# Human-robot-interaction metrics for the Stop motion cue
float64 human_approach_vel