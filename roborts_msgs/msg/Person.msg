# One detected person in the gimbal CSI camera image (P4).
std_msgs/Header header

# Stable tracking id (0 is never used).
uint32 id

# Bounding box in original image pixels.
geometry_msgs/Point center
geometry_msgs/Vector3 size

float32 score

# Distance estimate in metres. Assumes a person height of
# person_height_m (node parameter) and a calibrated camera.
float32 distance

# Bearing relative to the camera optical axis, in radians.
# yaw > 0 points right of the optical axis, pitch > 0 points down.
float32 yaw
float32 pitch
