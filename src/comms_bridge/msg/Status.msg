std_msgs/Header header

# Camera status codes

uint8 UNKNOWN = 0
uint8 NOT_FOUND = 1
uint8 CAMERA_CONNECTED = 2
uint8 RTSP_ERROR = 3
uint8 STREAM_CONNECTED = 4
uint8 STREAM_DISCONNECTED = 5

# Shredder status codes

uint8 SHREDDER_DISCONNECTED = 6
uint8 SHREDDER_CONNECTED = 7
uint8 SHREDDER_CONNECTING = 8

# Gimbal status codes

uint8 GIMBAL_DISCONNECTED = 9
uint8 GIMBAL_CONNECTED = 10


# Core fields

string name
uint8 status
float64 time
