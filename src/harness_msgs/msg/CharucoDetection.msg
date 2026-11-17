# The ChArUco board on one gripper-camera frame: tag_node publishes one on /camera/charuco with every frame (2026-11-16,
# the hand-eye target). Board frame = OpenCV's CharucoBoard frame: origin at the outer corner of square (0, 0) (the
# printed sheet's top-left), x along the squares_x squares, y along the squares_y squares (down the sheet), z into the
# board.
std_msgs/Header header          # frame_id = camera optical frame
bool seen                       # the pose was solved on this frame (>= min_corners corners, not all on one line)
int32 corners                   # inner chessboard corners found on this frame
int32 corners_total             # (squares_x - 1) x (squares_y - 1): 80 for the lab's sheet
int32 markers                   # ArUco markers decoded
string dictionary               # e.g. DICT_5X5_100
int32 squares_x
int32 squares_y
float64 square_m                # the square size the pose assumes
float64 marker_m
geometry_msgs/Pose pose         # the board frame in the camera optical frame (x right, y down, z forward); when seen
float64 distance_m              # camera -> the board's centre; when seen
float64 reproj_err_px           # mean corner reprojection error; when seen
int32[] corner_ids              # the corners found (OpenCV's ids: row by row from the top-left inner corner)
float64[] corners_px            # x0 y0 x1 y1 ... in corner_ids order
