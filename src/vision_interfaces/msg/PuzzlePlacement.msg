uint8 task_id
uint32 generation
uint32 piece_id
bool already_placed
bool workspace_mapping_valid
geometry_msgs/Point32 source_center_a4_mm
geometry_msgs/Point32 target_center_a4_mm
geometry_msgs/Point32 source_center_workspace_mm
geometry_msgs/Point32 target_center_workspace_mm
geometry_msgs/Point32 pick_point_workspace_mm
geometry_msgs/Point32 source_center_camera_px
geometry_msgs/Point32 target_center_camera_px
geometry_msgs/Point32[] target_polygon_a4_mm
geometry_msgs/Point32[] target_polygon_workspace_mm
# Dominant-edge orientations use the top-view convention: counter-clockwise positive.
# Because an edge is undirected, source/target angles are normalized to [-90, 90).
float32 source_angle_deg
float32 target_angle_deg
# Exact rigid rotation required from source to target, counter-clockwise positive,
# clockwise negative, normalized to [-180, 180). This is the serial angle field.
float32 rotation_delta_deg
float32 placement_score
