# One recognised hand gesture (P5).
#
# gesture_id:
#   0 none
#   1 fist
#   2 palm
#   3 victory
#   4 pointing_up
#   5 thumbs_up
std_msgs/Header header

# Tracking id of the person whose hand produced this gesture.
uint32 track_id

uint8 gesture_id
string name
float32 score

# Hand region in original image pixels.
geometry_msgs/Point center
geometry_msgs/Vector3 size
