byte HOME = 0
byte KAMAE = 1
byte HOLD = 2

byte MEN = 5
byte DOU = 6
byte KOTE = 7
byte TSUKI = 8
byte RDOU = 9

std_msgs/Header header
int32[] type
bool plan_result
string[] type_names
trajectory_msgs/JointTrajectory[] trajectories
