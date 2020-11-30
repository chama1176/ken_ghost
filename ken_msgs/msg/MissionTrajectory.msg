byte HOME = 0
byte KAMAE = 1

byte MEN = 5
byte DOU = 6
byte KOTE = 7
byte TSUKI = 8

std_msgs/Header header
int32[] type
bool plan_result
trajectory_msgs/JointTrajectory[] trajectories
