uint8 TYPE_STOP = 0
uint8 TYPE_MOTION = 1
uint8 TYPE_GOTO_FORWARD = 2
uint8 TYPE_GOTO_BACK = 3
uint8 type

uint8 MOTION_ID_L_SHAKE = 1
uint8 MOTION_ID_A_SHAKE = 2
uint8 MOTION_ID_TURN = 3
uint8 motion_id
float32 duration

float32 speed
geometry_msgs/Pose goal