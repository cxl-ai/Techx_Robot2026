# One complete vision frame decoded from one Jetson UDP V2 packet.
# target_count==0 and has_target==false means: bridge and Jetson are online, but no fresh target exists in this inference frame.

std_msgs/Header header
uint32 seq
uint8 protocol_version
float64 upstream_timestamp
uint8 target_count
bool has_target
techx_vision_bridge/VisionObject[] targets
