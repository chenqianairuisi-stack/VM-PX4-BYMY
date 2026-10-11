# Header copied unchanged from the source Image: exposure time, optical frame.
std_msgs/Header header
builtin_interfaces/Time processed_stamp
bool detected
uint32 image_width
uint32 image_height
# Normalized [0,1] image coordinates. All zero when detected is false.
float32 center_x
float32 center_y
float32 width
float32 height
float32 confidence
# True if the box touches an image boundary: size-based depth is unreliable.
bool clipped
