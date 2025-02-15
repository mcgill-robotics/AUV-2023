import rospy
import cv2
import numpy as np
import math
import quaternion
from tf import transformations
from sklearn.linear_model import RANSACRegressor
import json

from vision_state import VisionState


############## Utils Parameters ###############
MAX_DIST_TO_MEASURE = rospy.get_param("max_object_detection_distance")
BOX_COLOR = (255, 255, 255)  # White.
TEXT_COLOR = (0, 0, 0)  # Black.
# [COMP] ensure FOV is correct.
DOWN_CAM_HFOV = rospy.get_param("down_cam_hfov")
DOWN_CAM_VFOX = rospy.get_param("down_cam_vfov")
down_cam_x_offset = rospy.get_param("down_cam_x_offset")
DOWN_CAM_Y_OFFSET = rospy.get_param("down_cam_y_offset")
DOWN_CAM_Z_OFFSET = rospy.get_param("down_cam_z_offset")
DOWN_CAM_YAW_OFFSET = rospy.get_param("down_cam_yaw_offset")
MAX_COUNTS_PER_LABEL = json.loads(rospy.get_param("max_counts_per_label"))
states = (VisionState(), VisionState())
###############################################


def calculate_bbox_confidence(bbox, image_height, image_width):
    x_center, y_center, w, h = bbox

    x_center_offset = ((image_width / 2) - x_center) / image_width  # -0.5 to 0.5.
    y_center_offset = (y_center - (image_height / 2)) / image_height
    scaled_w = w / image_width
    scaled_h = h / image_height

    # ideal bounding box has centers at 0
    x_centering_score = 1 - abs(2 * x_center_offset)
    y_centering_score = 1 - abs(2 * y_center_offset)
    # ideal width is half of the image
    width_score = 1 - (2 * abs(0.5 - scaled_w))
    height_score = 1 - (2 * abs(0.5 - scaled_h))

    avg_score = (x_centering_score + y_centering_score + width_score + height_score) / 4

    return avg_score ** 2


# bbox is an array of 4 elements.
# Given an image, class name, and a bounding box, draws the
# bounding box rectangle and label name onto the image.
def visualize_bbox(image, bbox, class_name, thickness=2, fontSize=0.5):
    # Get xmin, xmax, ymin, ymax from bbox.
    x_center, y_center, w, h = bbox
    x_min = x_center - w / 2
    y_min = y_center - h / 2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    # Rectangle draws bounding box on image.
    cv2.rectangle(
        image, (x_min, y_min), (x_max, y_max), color=BOX_COLOR, thickness=thickness
    )
    # Get size of class name text.
    ((text_width, text_height), _) = cv2.getTextSize(
        class_name, cv2.FONT_HERSHEY_SIMPLEX, fontSize, 1
    )
    # Draw box around class name label on image.
    cv2.rectangle(
        image,
        (x_min, y_min - int(1.3 * text_height)),
        (x_min + text_width, y_min),
        BOX_COLOR,
        -1,
    )
    # Put class name text in the box we drew.
    cv2.putText(
        image,
        text=class_name,
        org=(x_min, y_min - int(0.3 * text_height)),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=fontSize,
        color=TEXT_COLOR,
        lineType=cv2.LINE_AA,
    )
    return image  # Returns inputted image with label.


def find_intersection(starting_point, vector, plane_z_pos):
    """
    Given a vector and a z position, returns the point
    where the vector intersects the plane defined by the z position.
    If they dont intersect, returns None.
    """
    if vector[2] == 0:
        return None

    z_diff = plane_z_pos - starting_point[2]

    scaling_factor = z_diff / vector[2]
    if scaling_factor < 0:
        return None

    return starting_point + np.array(vector) * scaling_factor


def get_object_position_down_camera(pixel_x, pixel_y, image_height, image_width, z_pos):
    """
    Given the pixel locations and height and width.

    Parameters:
        pixel_x: x coordinate of the object in the image.
        pixel_y: y coordinate of the object in the image.
        image_height: height of the image in pixels.
        image_width: width of the image in pixels.
        z_pos: z position of the object.

    Returns:
        x, y, z position in 3D space (not relative to the AUV).
    """
    # First calculate the relative offset of the object from the center of the
    # image (i.e. map pixel coordinates to values from -0.5 to 0.5).
    x_center_offset = ((image_width / 2) - pixel_x) / image_width  # -0.5 to 0.5.
    y_center_offset = (
        pixel_y - (image_height / 2)
    ) / image_height  # Negated since y goes from top to bottom.
    # Use offset within image and total FOV of camera to find
    # an angle offset from the angle the camera is facing
    # assuming FOV increases linearly with distance from center pixel.
    roll_angle_offset = DOWN_CAM_HFOV * x_center_offset
    pitch_angle_offset = DOWN_CAM_VFOX * y_center_offset

    x_pos_offset = -math.tan(math.radians(pitch_angle_offset))
    y_pos_offset = math.tan(math.radians(roll_angle_offset))

    local_direction_to_object = np.array([x_pos_offset, y_pos_offset, -1])

    rotation_offset = transformations.quaternion_from_euler(0, 0, DOWN_CAM_YAW_OFFSET)
    rotation = states[0].q_auv * np.quaternion(
        rotation_offset[3], rotation_offset[0], rotation_offset[1], rotation_offset[2]
    )
    global_direction_to_object = quaternion.rotate_vectors(
        rotation, local_direction_to_object
    )

    # Solve for point that is defined by the intersection of the
    # direction to the object and it's z position.
    global_down_cam_offset = quaternion.rotate_vectors(
        states[0].q_auv,
        np.array([down_cam_x_offset, DOWN_CAM_Y_OFFSET, DOWN_CAM_Z_OFFSET]),
    )
    down_cam_pos = (
        np.array([states[0].position.x, states[0].position.y, states[0].position.z])
        + global_down_cam_offset
    )
    obj_pos = find_intersection(down_cam_pos, global_direction_to_object, z_pos)

    if (
        obj_pos is None
        or np.linalg.norm(
            obj_pos
            - np.array(
                [states[0].position.x, states[0].position.y, states[0].position.z]
            )
        )
        > MAX_DIST_TO_MEASURE
    ):
        return None, None, None
    x = obj_pos[0]
    y = obj_pos[1]
    z = z_pos
    return x, y, z


# Given a bounding box, tells you where the main object in the
# bounding box is in 3D space (world space).
# Assumes cleaning was correct.
def get_object_position_front_camera(bbox):
    point_cloud = states[1].get_point_cloud(bbox)
    min_lx = np.nanmin(point_cloud[:, :, 0].flatten())
    min_ly = np.nanmin(point_cloud[:, :, 1].flatten())
    min_lz = np.nanmin(point_cloud[:, :, 2].flatten())
    max_lx = np.nanmax(point_cloud[:, :, 0].flatten())
    max_ly = np.nanmax(point_cloud[:, :, 1].flatten())
    max_lz = np.nanmax(point_cloud[:, :, 2].flatten())

    lx = (max_lx + min_lx) / 2
    ly = (max_ly + min_ly) / 2
    lz = (max_lz + min_lz) / 2

    global_obj_pos_offset = quaternion.rotate_vectors(
        states[1].q_auv, np.array([lx, ly, lz])
    )

    # Get the best estimate of the mean.
    x, y, z = global_obj_pos_offset + np.array(
        [states[1].position.x, states[1].position.y, states[1].position.z]
    )
    
    return x, y, z


# Tells you how the object is oriented in space (Z axis).
def measure_angle(bbox):
    """
    Given a bounding box, returns the angle of the object in degrees (only for front cam)
    """
    point_cloud = states[1].get_point_cloud(bbox)

    point_cloud_x = point_cloud[
        :, :, 0
    ].flatten()  # Collect x, ignore z positions of points.
    point_cloud_y = point_cloud[
        :, :, 1
    ].flatten()  # Collect y, ignore z position of points.

    nan_indices = np.isnan(point_cloud_x) | np.isnan(
        point_cloud_y
    )  # Find indices to remove (NaNs).

    filtered_point_cloud_x = point_cloud_x[
        ~nan_indices
    ]  # Filter point cloud x so it has no NaNs.
    filtered_point_cloud_y = point_cloud_y[
        ~nan_indices
    ]  # Filter point cloud y so it has no NaNs.

    # Fit a line to the point cloud X/Y with RANSAC (removes outliers).
    ransac = RANSACRegressor()
    ransac.fit(-1 * filtered_point_cloud_y.reshape(-1, 1), filtered_point_cloud_x)
    slope = ransac.estimator_.coef_[0]

    angle = math.degrees(math.atan(slope))  # Calculate the angle of the fitted line.

    return angle + states[1].theta_z


# Lots of noise in pool, the idea is for example if the down
# cam has two detections, it will remove the least confident one.
# Selects highest confidence detection from duplicates and ignores
# objects with no position measurement.
def clean_detections(detectionFrameArray):
    label_counts = {}
    selected_detections = []

    for i in range(len(detectionFrameArray)):
        obj = detectionFrameArray[i]
        if None in [obj.x, obj.y, obj.z]:
            continue
        if label_counts.get(obj.label, 0) >= MAX_COUNTS_PER_LABEL[obj.label]:
            candidate_obj_conf = obj.confidence
            min_conf_i = min(
                selected_detections, key=lambda x: detectionFrameArray[x].confidence
            )
            if detectionFrameArray[min_conf_i].confidence < candidate_obj_conf:
                selected_detections.remove(min_conf_i)
                selected_detections.append(i)
        else:
            label_counts[obj.label] = label_counts.get(obj.label, 0) + 1
            selected_detections.append(i)

    return [detectionFrameArray[i] for i in selected_detections]


# Automatically import all functions and constants.
__all__ = [name for name in globals() if not name.startswith("__")]
