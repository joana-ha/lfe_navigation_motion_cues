# Message that contains logs for BackOff motion cue
# Header seq and frame_id are retrieved from Body Image Frame from Astra SDK, timestamp is the current time
std_msgs/Header header

# Technical BackOff parameter, id, duration in seconds, velocity in m/s, coordinates where robot stops
std_msgs/String id
bool cross_situation
float64 back_duration
float64 back_velocity
float64 wait_seconds

geometry_msgs/Point32 robot_coord

float64 robot_lin_vel_x

# Human-robot-interaction metrics for the BackOff motion cue
bool human_continue
float64 human_approach_vel
float64[20] human_approach_dist_seq
float64[20] human_approach_dist_time_seq
float64[100] human_mc_dist_seq
float64[100] human_mc_dist_time_seq
float64 time4interaction