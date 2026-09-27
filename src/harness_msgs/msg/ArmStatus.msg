# Published by harness_arm at the feedback rate. The single source of truth for arm state.
std_msgs/Header header
bool connected            # CAN feedback is arriving
bool ok                   # connected, no error bits, not in teach mode
string ctrl_mode          # STANDBY | CAN_CTRL | TEACH | ...
string arm_status         # NORMAL | ESTOP | NO_SOLUTION | ... (controller's own words)
string motion_status
bool teach_mode           # drag-teach active: motion commands are refused
bool[6] joints_enabled    # from joint feedback, never from an SDK return value
bool all_enabled
string[] errors           # error-bit names that are set
float64 feedback_age_s    # seconds since the last feedback frame
float64 feedback_hz
int32 speed_percent       # last commanded speed cap
bool busy                 # a motion goal is executing
string fault              # non-empty after a watchdog/error auto-disable; cleared by /arm/enable
