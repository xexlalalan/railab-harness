int32 id
string family
float64 size_m                 # edge length assumed for the pose
float64[2] center_px
float64[8] corners_px          # x0 y0 x1 y1 x2 y2 x3 y3 (top-left, top-right, bottom-right, bottom-left)
float64 size_px                # mean edge length in pixels
geometry_msgs/Pose pose        # tag frame in the camera optical frame (x right, y down, z forward)
float64 distance_m
float64 reproj_err_px
bool in_center_square          # centre inside the central square AND size within [min,max] px
float64 depth_m                # median stereo depth over the tag area (0 = none); independent of the assumed size
float64 depth_size_m           # black-square edge implied by depth: size_m * depth_m / distance_m (0 = none); depth is unreliable below ~100 mm
