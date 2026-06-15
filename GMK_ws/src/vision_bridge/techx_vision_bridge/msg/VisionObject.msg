# One detected object decoded from one Jetson vision frame.
# Four fixed frames are used:
#   camera_link  : raw x/y/z from Jetson RGB-D camera, meters.
#   robot_base   : robot body frame after T_robot_camera.
#   arm1_base    : arm 1 base frame after T_arm1_robot.
#   arm2_base    : arm 2 base frame after T_arm2_robot.
# control_x/y/z are the recommended coordinates for the object's consumer.
# New object types can use custom numeric target_type/zone_id values configured
# in vision_bridge.yaml class_rules. Existing consumers should always filter by
# target_type/class_id instead of relying on receive order.

uint8 ZONE_UNKNOWN=0
uint8 ZONE_WEAPON_HEAD=1
uint8 ZONE_KFS=2
uint8 ZONE_QR=3
uint8 ZONE_CUSTOM=10

uint8 TYPE_UNKNOWN=0
uint8 TYPE_WEAPON_HEAD=1
uint8 TYPE_KFS=2
uint8 TYPE_QR=3
uint8 TYPE_CUSTOM=10

uint8 COLOR_UNKNOWN=0
uint8 COLOR_RED=1
uint8 COLOR_BLUE=2

uint8 FRAME_UNKNOWN=0
uint8 FRAME_CAMERA_LINK=1
uint8 FRAME_ROBOT_BASE=2
uint8 FRAME_ARM1_BASE=3
uint8 FRAME_ARM2_BASE=4

std_msgs/Header header
uint32 seq
uint8 target_index
uint8 target_count

uint8 zone_id
uint8 target_type
uint8 class_id
uint8 color
float32 confidence

float32 u
float32 v

bool valid_xyz
float32 x
float32 y
float32 z

bool valid_robot_xyz
float32 robot_x
float32 robot_y
float32 robot_z

bool valid_arm1_xyz
float32 arm1_x
float32 arm1_y
float32 arm1_z

bool valid_arm2_xyz
float32 arm2_x
float32 arm2_y
float32 arm2_z

uint8 control_frame
bool valid_control_xyz
float32 control_x
float32 control_y
float32 control_z

float32 align_err_x
float32 align_err_y
float32 priority
