# /<instance>/state - pushed by a module node on every change and at the idle beat (so it is the
# module's heartbeat too). Module interface, 2026-11-20.
#
# What every module has is typed (`ros2 topic echo` reads it, the harness needs nothing else);
# what is the module's own travels as JSON.
# The JSON form of the contract names the instance "module"; here it is "instance" ("module" is a
# reserved word of OMG IDL, which rosidl turns every .msg into).

builtin_interfaces/Time stamp
string instance           # the instance = its namespace, e.g. "bunny_1"

string mode               # not_ready | ready | busy | fault
string fault_code         # "" = no fault. Latched until the action clear_fault.
string fault_reason

bool has_current          # an action is running
int64 current_id
string current_action
float32 current_progress  # 0 ... 1

bool has_last             # the result of the action that ended last
int64 last_id
string last_action
string last_result        # done | stopped | failed
string last_reason        # "" when there is nothing to say

string describe           # JSON: {identity {type, serial, version}, actions, fields, zones}
string values             # JSON: {field: value | "unknown"}
string sources            # JSON: {field: "sensed" | "told" | "estimated" | "unknown"}
