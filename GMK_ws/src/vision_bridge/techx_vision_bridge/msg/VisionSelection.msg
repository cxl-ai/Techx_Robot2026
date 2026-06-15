# Best target selected by techx_vision_bridge from the latest VisionFrame
# according to the latest VisionRequest.

uint8 STATUS_OK=0
uint8 STATUS_NO_REQUEST=1
uint8 STATUS_NO_FRAME=2
uint8 STATUS_NO_MATCH=3
uint8 STATUS_FRAME_STALE=4
uint8 STATUS_REQUEST_STALE=5

std_msgs/Header header
uint32 frame_seq
uint32 request_seq
bool has_request
bool has_match
uint8 status
uint8 selected_index
float32 frame_age_sec
float32 score
techx_vision_bridge/VisionObject target
