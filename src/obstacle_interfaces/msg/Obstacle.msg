# A single detected obstacle.
# 3D fields are expressed in the frame of the enclosing ObstacleArray (os_sensor:
# +x front, +y left, +z up).

uint8 SOURCE_LIDAR=0
uint8 SOURCE_IR=1
uint8 SOURCE_FUSED=2
uint8 SOURCE_RGB=3

uint32 id
uint8 source
string label                    # YOLO class ("car", "person"...) or "unknown"
float32 confidence              # 0..1

bool has_3d                     # true if the 3D fields below are valid
geometry_msgs/Point center
geometry_msgs/Vector3 size      # axis-aligned box, m
float32 distance                # horizontal distance to the sensor, m
float32 azimuth                 # rad; 0 = front, positive = left
uint32 num_points

bool has_bbox                   # box in the source image: IR 1024x128 (RGB 1920x1200 if source=RGB)
uint16 u_min
uint16 v_min
uint16 u_max
uint16 v_max
