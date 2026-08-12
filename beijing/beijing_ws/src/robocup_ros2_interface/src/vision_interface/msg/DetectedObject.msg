string label
string color
float64 confidence
int64 xmin
int64 ymin
int64 xmax
int64 ymax
float32[] target_uv
float32[] received_pos
float32[] position
float32[] position_projection
# Ground-plane projections of the detected object's bbox bottom-left and
# bottom-right corners.  Values use the robot base frame and metres.
float32[] position_projection_left
float32[] position_projection_right
float32[] position_cam
int32 position_confidence
bool fallen
float64 fallen_confidence
