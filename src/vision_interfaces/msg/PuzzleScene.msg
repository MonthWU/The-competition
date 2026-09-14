std_msgs/Header header
uint8 task_id
uint32 generation
bool a4_detected
bool valid
bool workspace_mapping_valid
string status
float64[9] a4_to_workspace_homography
float64[9] a4_to_image_homography
geometry_msgs/Point32[4] a4_corners_workspace_mm
uint32 image_width
uint32 image_height
# Green-frame center in the original image with the camera center as origin.
# X points right and Y points down.
geometry_msgs/Point32 green_frame_center_camera_px
# Top-left A4-plane coordinate represented by rectified_image. All current tasks
# use the portrait A4 page itself, so this is (0,0); kept for interface compatibility.
geometry_msgs/Point32 rectified_origin_a4_mm
vision_interfaces/PuzzlePiece[] pieces
sensor_msgs/Image rectified_image
