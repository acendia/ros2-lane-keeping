# Lane geometry in vehicle coordinates.
# The header timestamp is the camera frame time used for the estimate.
std_msgs/Header header

# False when no plausible pair of lane boundaries was found.
bool valid

# Detector confidence in [0.0, 1.0].
float32 confidence

# Signed lateral offset of the lane center at the lookahead distance, in metres.
float32 lateral_error

# Lane-center target about 4 m ahead in vehicle coordinates (x forward, y left).
geometry_msgs/Point lookahead_target
