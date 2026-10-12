std_msgs/Header header
string bus_id
string state                # IDLE, APPROACH, WAIT, DOCK, BOARD, COMPLETE, SAFE_STOP
float32 eta_sec             # -1 when eta_valid=false
bool eta_valid
bool door_open
