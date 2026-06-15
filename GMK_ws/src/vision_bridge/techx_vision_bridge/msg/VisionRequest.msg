# Request/filter sent by a decision package to use the bridge as a smart vision sensor.
# Publish to /techx/vision/request. The selector keeps the latest request and
# publishes the best matching target on /techx/vision/selected whenever a fresh
# vision frame arrives, and periodically publishes stale/no-match status.

uint8 TYPE_ANY=0
uint8 TYPE_WEAPON_HEAD=1
uint8 TYPE_KFS=2
uint8 TYPE_QR=3
uint8 TYPE_CUSTOM=10

uint8 ZONE_ANY=0
uint8 ZONE_WEAPON_HEAD=1
uint8 ZONE_KFS=2
uint8 ZONE_QR=3
uint8 ZONE_CUSTOM=10

std_msgs/Header header

# Monotonic sequence from decision package. It is echoed by VisionSelection.
uint32 request_seq

# Primary filter. Use 0 to accept any type/zone.
uint8 target_type
uint8 zone_id

# Optional exact class/color filters.
bool use_class_id
uint8 class_id
bool use_color
uint8 color

# If true, selected target must have valid control_x/y/z.
bool require_control_xyz

# Minimum confidence. Use 0.0 to disable.
float32 min_confidence

# Maximum acceptable frame age in seconds when the request is evaluated.
# Use 0.0 to use the selector default.
float32 max_frame_age_sec
