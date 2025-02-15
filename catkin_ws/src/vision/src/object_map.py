#!/usr/bin/env python3

import rospy
import math

from auv_msgs.msg import VisionObject, VisionObjectArray


# Callback when a new object detection frame is published.
def object_detect_cb(msg):
    try:
        add_observation(msg)
        reduce_map()
        publish_map()
    except Exception as e:
        print(str(e))


# Add an object detection frame to the object map.
def add_observation(msg):
    # Loop over every object in the detection_frame array.
    for detection_frame in msg.array:
        # Find which object this detection pertains to.
        obj_i = find_closest_object(
            [
                detection_frame.label,
                detection_frame.x,
                detection_frame.y,
                detection_frame.z,
            ]
        )
        # If it does not pertain to any preexisting object add it to the map.
        if obj_i == -1:
            object_map.append(
                [
                    detection_frame.label,
                    detection_frame.x,
                    detection_frame.y,
                    detection_frame.z,
                    detection_frame.theta_z,
                    detection_frame.extra_field,
                    1,
                    detection_frame.confidence,
                ]
            )
        else:
            # Otherwise update the object map with the new observation.
            update_map(
                obj_i,
                [
                    detection_frame.label,
                    detection_frame.x,
                    detection_frame.y,
                    detection_frame.z,
                    detection_frame.theta_z,
                    detection_frame.extra_field,
                    1,
                    detection_frame.confidence,
                ],
            )


# Given an observation, find the object to which it pertains
# to (object within a certain radius of same class).
def find_closest_object(observation, indexToIgnore=-1):
    observed_label, observed_x, observed_y, observed_z = observation
    # Find all objects within sameObjectRadius of this observation in the map.
    close_objs = []
    # Go through every object in map and add to close_objs if close enough.
    for obj_i in range(len(object_map)):
        obj_label, obj_x, obj_y, obj_z, _, _, _, _ = object_map[obj_i]
        if observed_label != obj_label or obj_i == indexToIgnore:
            continue
        # Find distance between object in map and observation.
        # Ignore Z position when reducing map.
        objs_distance_apart = (
            dist((obj_x, obj_y, obj_z), (observed_x, observed_y, observed_z))
            if indexToIgnore == -1
            else dist((obj_x, obj_y, 0), (observed_x, observed_y, 0))
        )
        if objs_distance_apart < same_object_radius_per_label.get(observed_label, 1000):
            close_objs.append(obj_i)
    # If there is only one object within radius return that.
    if len(close_objs) == 0:
        return -1
    # Return closest object to observation.
    else:
        return min(
            close_objs,
            key=lambda i: dist(
                (object_map[i][1], object_map[i][2], object_map[i][3]),
                (observed_x, observed_y, observed_z),
            ),
        )


def angle_difference(ang1, ang2):
    diff = (ang1 - ang2) % 360
    if diff > 180:
        diff -= 360
    return abs(diff)


# @TO-DO: Check probabilities.
# Update the object map using probabilities to improve
# estimate of object pose and label.
def update_map(obj_i, observation):
    (
        _,
        observed_x,
        observed_y,
        observed_z,
        observed_theta_z,
        observed_extra_field,
        num_new_observations,
        observed_confidence,
    ) = observation
    (
        label,
        current_x,
        current_y,
        current_z,
        current_theta_z,
        current_extra_field,
        num_observations,
        current_confidence,
    ) = object_map[obj_i]

    if observed_confidence <= 0:
        observed_confidence = 1e-5
    if current_confidence <= 0:
        current_confidence = 1e-5

    # Calculate pose.
    new_x = (
        observed_confidence * observed_x
        +  current_confidence * current_x
    ) / (
         current_confidence
        + observed_confidence
    )
    new_y = (
        observed_confidence * observed_y
        +  current_confidence * current_y
    ) / (
         current_confidence
        + observed_confidence
    )
    new_z = (
        observed_confidence * observed_z
        +  current_confidence * current_z
    ) / (
         current_confidence
        + observed_confidence
    )

    if label == "Lane Marker":
        # Lane marker had to dealt with differently for theta z and extra_field.
        # If no theta z measurement keep current theta z.
        if (
            observed_theta_z == NULL_PLACEHOLDER
            and observed_extra_field == NULL_PLACEHOLDER
        ):
            new_theta_z = current_theta_z
            new_extra_field = current_extra_field
        # If there was no previous theta z but observation has a theta z set
        # theta z to observation theta z.
        elif (
            current_extra_field == NULL_PLACEHOLDER
            and current_theta_z == NULL_PLACEHOLDER
        ):
            new_theta_z = observed_theta_z
            new_extra_field = observed_extra_field
        else:
            # Find closest match between observed angles and current angles.
            if angle_difference(observed_theta_z, current_theta_z) < angle_difference(
                observed_theta_z, current_extra_field
            ):
                # Closest angles are observed theta_z and current theta_z.
                while abs(observed_theta_z - current_theta_z) > 180:
                    if observed_theta_z > current_theta_z:
                        observed_theta_z -= 360
                    else:
                        observed_theta_z += 360
                new_theta_z = (
                    observed_confidence * observed_theta_z
                    +  current_confidence * current_theta_z
                ) / (
                     current_confidence
                    + observed_confidence
                )
                # Closes angles are observed extra_field and current extra_field.
                while abs(observed_extra_field - current_extra_field) > 180:
                    if observed_extra_field > current_extra_field:
                        current_extra_field += 360
                    else:
                        observed_extra_field += 360
                new_extra_field = (
                    observed_confidence * observed_extra_field
                    +  current_confidence * current_extra_field
                ) / (
                     current_confidence
                    + observed_confidence
                )
            else:
                # Closest angles are observed theta_z and current extra_field.
                while abs(observed_theta_z - current_extra_field) > 180:
                    if observed_theta_z > current_extra_field:
                        observed_theta_z -= 360
                    else:
                        observed_theta_z += 360
                new_extra_field = (
                    observed_confidence * observed_theta_z
                    +  current_confidence * current_extra_field
                ) / (
                     current_confidence
                    + observed_confidence
                )
                # Closest angles are observed extra_field and current theta_z.
                while abs(observed_extra_field - current_theta_z) > 180:
                    if observed_extra_field > current_theta_z:
                        observed_extra_field -= 360
                    else:
                        observed_extra_field += 360
                new_theta_z = (
                    observed_confidence * observed_extra_field
                    +  current_confidence * current_theta_z
                ) / (
                     current_confidence
                    + observed_confidence
                )
    else:
        # Calculate theta z.
        # if no theta z measurement keep current theta z.
        if observed_theta_z == NULL_PLACEHOLDER:
            new_theta_z = current_theta_z
        # If there was no previous theta z but observation has
        # a theta z set theta z to observation theta z.
        elif current_theta_z == NULL_PLACEHOLDER:
            new_theta_z = observed_theta_z
        else:
            while abs(observed_theta_z - current_theta_z) > 180:
                if observed_theta_z > current_theta_z:
                    current_theta_z += 360
                else:
                    observed_theta_z += 360
            # Average both orientations.
            new_theta_z = (
                observed_confidence * observed_theta_z
                +  current_confidence * current_theta_z
            ) / (
                 current_confidence
                + observed_confidence
            )

        # Calculate extra_field when applicable.
        if label == "Gate":  # Gate, symbol on left (0 or 1) -> take weighted average.
            if observed_extra_field == NULL_PLACEHOLDER:
                new_extra_field = current_extra_field
            elif current_extra_field == NULL_PLACEHOLDER:
                new_extra_field = observed_extra_field
            else:
                new_extra_field = (
                    observed_confidence * observed_extra_field
                    +  current_confidence * current_extra_field
                ) / (
                     current_confidence
                    + observed_confidence
                )
        else:
            new_extra_field = NULL_PLACEHOLDER

    # calculate the new confidence as the probability that neither of the existing or new observation are incorrect
    p_incorrect_current = 1 - current_confidence
    p_incorrect_observed = 1 - observed_confidence
    p_correct_current = current_confidence
    p_correct_observed = observed_confidence

    p_incorrect = p_incorrect_current * p_incorrect_observed
    p_imperfect = p_correct_current * p_incorrect_observed
    p_imperfect += p_incorrect_current * p_correct_observed

    # Assume that imperfect readings cause incorrect estimates 25% of the time
    p_incorrect += 0.25 * p_imperfect

    new_confidence = 1.0 - p_incorrect

    # limit confidence to 50% above the highest confidence (observation or current)
    # new_confidence = min(1.5 * max(observed_confidence, current_confidence), new_confidence)

    object_map[obj_i][1] = new_x
    object_map[obj_i][2] = new_y
    object_map[obj_i][3] = new_z
    object_map[obj_i][4] = new_theta_z
    object_map[obj_i][5] = new_extra_field
    object_map[obj_i][6] += num_new_observations
    object_map[obj_i][7] = new_confidence


# Calculate euclidian distance between two objects.
def dist(obj1, obj2):
    x1, y1, z1 = obj1
    x2, y2, z2 = obj2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


# Combine similar/close objects in the map into one object.
def reduce_map():
    num_objs_deleted = 0
    for i in range(len(object_map)):
        idx = i - num_objs_deleted
        observed_label, observed_x, observed_y, observed_z, _, _, _, _ = object_map[idx]
        closest_obj = find_closest_object(
            [observed_label, observed_x, observed_y, observed_z], indexToIgnore=idx
        )
        if closest_obj == -1:
            continue
        else:
            update_map(closest_obj, object_map[idx])
            del object_map[i]
            num_objs_deleted += 1


# Publish a version of the map with only the objects
# with a certain number of observations.
def publish_map():
    confirmedMap = [obj for obj in object_map if obj[6] > MIN_OBSERVATIONS]

    # Create an array of ObjectMap.
    map_msg_array = VisionObjectArray()
    for obj in confirmedMap:
        map_msg = VisionObject()
        map_msg.label = obj[0]
        map_msg.x = obj[1]
        map_msg.y = obj[2]
        map_msg.z = obj[3]
        map_msg.theta_z = obj[4]
        map_msg.extra_field = obj[5]
        map_msg.confidence = obj[7]
        map_msg_array.array.append(map_msg)
    obj_pub.publish(map_msg_array)


if __name__ == "__main__":
    rospy.init_node("object_map")

    MIN_OBSERVATIONS = rospy.get_param("min_observations_for_mapping")
    object_map = []

    NULL_PLACEHOLDER = rospy.get_param("NULL_PLACEHOLDER")

    # In same units as state_x, y, z etc (only for
    # objects which can appear more than once).
    same_object_radius_per_label = {
        "Lane Marker": rospy.get_param("same_object_radius_lane_marker")
    }

    obj_pub = rospy.Publisher("vision/object_map", VisionObjectArray, queue_size=1)

    obj_sub = rospy.Subscriber(
        "vision/viewframe_detection", VisionObjectArray, object_detect_cb
    )

    rospy.spin()
