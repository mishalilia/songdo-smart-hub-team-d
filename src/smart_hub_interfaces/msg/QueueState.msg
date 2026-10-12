std_msgs/Header header
string zone_id
uint32 count
float32 confidence          # [0, 1]; low/stale => valid=false (UNKNOWN), never "zero people"
bool valid
bool crossing_occupied
