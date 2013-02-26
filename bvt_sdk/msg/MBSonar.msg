Header header			#The usual message header information

uint8 RTHETA_IMAGE=0
uint8 XY_IMAGE=1

sensor_msgs/Image image #The sensor image i R-Theta or X-Y layout
float32 range_res #The range resolution
float32 bearing_res #The bearing resolution
uint8 type #R-Theta or X-Y