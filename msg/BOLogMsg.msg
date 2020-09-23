# Message that contains logs for BackOff motion cue
# Header seq and frame_id are retrieved from Body Image Frame from Astra SDK, timestamp is the current time
std_msgs/Header header

# Technical BackOff parameter, id, duration in seconds, velocity in m/s, coordinates where robot stops
std_msgs/String id
float64 back1_duration
float64 forw_duration
float64 back2_duration
float64 total_duration
float64 wait_seconds

float64 back1_velocity
float64 forw_velocity
float64 back2_velocity

geometry_msgs/Point32 robot_coord

float64 robot_lin_vel_x

# Human-robot-interaction metrics for the BackOff motion cue
float64 human_approach_vel