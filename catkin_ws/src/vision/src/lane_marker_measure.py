#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge

from common_utils import crop_to_bbox
from sensor_msgs.msg import Image


############## Utils Parameters ###############
LONGEST_DOWNSCALED_SIZE = rospy.get_param("lane_marker_downscaling_size")
BLUR1_AMT = rospy.get_param("lane_marker_blur_1_amt")
BLUR2_AMT = rospy.get_param("lane_marker_blur_2_amt")
COLOR_TOLERANCE = rospy.get_param("lane_marker_color_tolerance")
TESTING = rospy.get_param("testing", False)

HEADING_COLOR = (255, 0, 0)  # Blue
bridge = CvBridge()

pub_cropped_image = rospy.Publisher("/vision/down_cam/cropped", Image, queue_size=1)
###############################################


# Checks if color1 similar enough (within tolerance) to color2.
# Assumes color is in BGR.
def colors_are_equal(color1, color2):
    # Make color brightnesses the same (just care about color ratios, 
    # not brightness/saturation) by making max value 255 in any channel.
    color1 += min((255, 255, 255) - color1)
    color2 += min((255, 255, 255) - color2)
    for i in (0, 1, 2):
        if max(color1[i], color2[i]) - min(color1[i], color2[i]) > COLOR_TOLERANCE * 255:
            return False
    return True


# Receives a cv2 image and returns a black and white cv2 image 
# where the "reddest" pixels are black.
def threshold_red_to_black(
    image,
    downscale_publisher=None,
    blur1_publisher=None,
    tol_publisher=None,
    blur2_publisher=None,
    thresh_publisher=None,
):
    if min(image.shape[0], image.shape[1]) > LONGEST_DOWNSCALED_SIZE:
        scaling_factor = LONGEST_DOWNSCALED_SIZE / min(image.shape[0], image.shape[1])
        downscaled_size = (
            int(image.shape[1] * scaling_factor),
            int(image.shape[0] * scaling_factor),
        )
    else:
        downscaled_size = (image.shape[1], image.shape[0])
    downscaled = cv2.resize(image, dsize=downscaled_size, interpolation=cv2.INTER_AREA)

    if BLUR1_AMT > 0:
        blurred = cv2.blur(
            downscaled,
            (
                max(2, int(BLUR1_AMT * downscaled.shape[0])),
                max(2, int(BLUR1_AMT * downscaled.shape[1])),
            ),
        )
    else:
        blurred = downscaled
    image_b, image_g, image_r = cv2.split(blurred)
    # Normalize colors so that the max color channel in every pixel is 255.
    max_brightness_increase = np.min(255 - blurred, axis=2)
    ratio_image = np.uint32(image_r + max_brightness_increase) / (
        np.uint32(image_g + max_brightness_increase)
        + np.uint32(image_b + max_brightness_increase)
        + np.uint32(image_r + max_brightness_increase)
    )
    max_i = np.argmax(ratio_image)  # Get largest value in red color channel.
    max_pixel = blurred[math.floor(max_i / blurred.shape[1])][max_i % blurred.shape[1]]
    if downscale_publisher != None:
        downscale_publisher.publish(
            bridge.cv2_to_imgmsg(downscaled, "bgr8")
        )  # For adjusting values.
    if blur1_publisher != None:
        blur1_publisher.publish(
            bridge.cv2_to_imgmsg(blurred, "bgr8")
        )  # For adjusting values.

    def mask(x):
        # Return black (0,0,0) if similar to max_red within tolerance.
        # Otherwise, return white (255,255,255).
        if colors_are_equal(x, max_pixel):
            return (0, 0, 0)
        else:
            return (255, 255, 255)

    thresh_image = np.uint8(np.zeros(blurred.shape))
    for r in range(len(blurred)):
        for p in range(len(blurred[r])):
            thresh_image[r][p] = mask(blurred[r][p])

    dilation_kernel = np.ones((5, 5), np.uint8)
    inverted_thresh_image = cv2.bitwise_not(thresh_image)
    while (
        np.sum(inverted_thresh_image)
        / (255 * 3 * inverted_thresh_image.shape[0] * inverted_thresh_image.shape[1])
        < 0.2
    ):
        inverted_thresh_image = cv2.dilate(
            inverted_thresh_image, dilation_kernel, iterations=1
        )
    thresh_image = cv2.bitwise_not(inverted_thresh_image)

    if tol_publisher != None:
        tol_publisher.publish(
            bridge.cv2_to_imgmsg(thresh_image, "bgr8")
        )  # For adjusting values.

    thresh_image = cv2.resize(
        thresh_image,
        dsize=(int(image.shape[1]), int(image.shape[0])),
        interpolation=cv2.INTER_AREA,
    )
    if BLUR2_AMT > 0:
        thresh_image = cv2.blur(
            thresh_image,
            (
                max(2, int(BLUR2_AMT * thresh_image.shape[0])),
                max(2, int(BLUR2_AMT * thresh_image.shape[1])),
            ),
        )
    if blur2_publisher != None:
        blur2_publisher.publish(
            bridge.cv2_to_imgmsg(thresh_image, "bgr8")
        )  # For adjusting values.

    thresh_image = cv2.cvtColor(
        thresh_image, cv2.COLOR_BGR2GRAY
    )  # Convert image to grayscale.
    ret, thresh_image = cv2.threshold(
        thresh_image, 70, 255, 0
    )  # Convert grayscale to black and white with a threshold.

    if thresh_publisher != None:
        thresh_publisher.publish(
            bridge.cv2_to_imgmsg(thresh_image, "mono8")
        )  # For adjusting values.

    return thresh_image


# Return the intersection point of two lines of the form (slope, y intercept).
def get_lines_intersection(l1, l2):
    # If m, m' are slopes and b, b' are intercepts the intersection is at x which satisfies:
    # mx+b=m'x+b'.
    # mx-m'x=b'-b.
    # x=(b'-b)/(m-m').
    if l1[0] - l2[0] == 0:
        return None
    int_x = (l2[1] - l1[1]) / (l1[0] - l2[0])
    # y=mx+b.
    int_y = int_x * l2[0] + l2[1]
    return (int(int_x), int(int_y))


# Given an array of the form 
# ((slope1UpperLine, slope1LowerLine), (slope2UpperLine, slope2LowerLine)),
# it return the center point of the rectangle defined by the 4 lines.
def get_rectangle_center_point(lines):
    # Get intersection between the two upper lines.
    int1 = get_lines_intersection(lines[0][0], lines[1][0])
    # Get intersection between the two lower lines.
    int2 = get_lines_intersection(lines[0][1], lines[1][1])
    if None in (int1, int2):
        return None
    # Get point halfway between the two intersections.
    centerPoint = (int((int1[0] + int2[0]) / 2), int((int1[1] + int2[1]) / 2))
    return centerPoint


def angle_between_lines(l1, l2):
    l1_angle = 180 * math.atan(l1[0]) / math.pi  # Between -90 and 90.
    l2_angle = 180 * math.atan(l2[0]) / math.pi  # Between -90 and 90.
    angle_diff = abs(l1_angle - l2_angle)  # Between 0 and 180.
    if angle_diff > 90:
        return 180.0 - angle_diff
    else:
        return angle_diff


# Given an image containing a lane marker, returns one slope per lane 
# marker heading (relative to the image's x/y axis) (i.e, it should return two slopes).
# Returns lines have the format (l1, l2) where l1 is the heading that is closest to that 
# of the AUV, and l2 is the heading where the AUV should go should also return center point 
# of lane marker (or most central point if not completely contained in image)).
def measure_headings(image, debug=False, debug_image=None):
    if debug_image is None:
        debug_image = image
    if debug:
        cv2.imshow("original", image)
        cv2.waitKey(0)
    thresh_image = threshold_red_to_black(image)
    if debug:
        cv2.imshow("thresholded/black and white", thresh_image)
        cv2.waitKey(0)
    # Get edges of thresholded image (should get the edges of the lane marker).
    edges = cv2.Canny(thresh_image, 50, 150, apertureSize=3)
    lines = []
    # Only want up to 4 slopes (one per side of the lane marker).
    while len(lines) < 4:
        if debug:
            cv2.imshow("remaining edges", edges)
            cv2.waitKey(0)
            pass
        # Find most prominent line in the image.
        line = cv2.HoughLines(edges, 1, np.pi / 180, 25)
        # We use a try statement in case no lines are found
        # when no lines are found a typeerror is thrown.
        try:
            for rho, theta in line[0]:
                # Compute line coordinates in image using rho and theta.
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 3000 * (-b))
                y1 = int(y0 + 3000 * (a))
                x2 = int(x0 - 3000 * (-b))
                y2 = int(y0 - 3000 * (a))
                # Calculate slope from start and end points of line.
                if (x2 - x1) == 0:
                    slope = 1000000
                else:
                    slope = (y2 - y1) / (x2 - x1)
                # y = mx+b.
                # Slope is m, intercept is b.
                # Therefore the slope is b = y-mx for any point on the line.
                intercept = y1 - slope * x1
                lines.append((slope, intercept))
                # Draw the new line we found on top of the original image.
                # Remove the line from the edges image by drawing the line with 
                # extra thickness. This covers up the line that was detected 
                # (edges are in white, the line is drawn in black).
                line_thickness = max(int(0.05 * min(edges.shape)), 1)
                cv2.line(edges, (x1, y1), (x2, y2), (0, 0, 0), line_thickness)
                cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        except TypeError:
            break
    if debug:
        cv2.imshow("remaining edges", edges)
        cv2.waitKey(0)
        pass

    if len(lines) < 2:
        return None, None
    elif len(lines) < 4:
        # If there arent 4 lines we can assume the lane marker is 
        # straight i.e. we only consider two lines.
        if len(lines) == 3:
            # Remove line that is most different from the other two.
            l0l1_angle = angle_between_lines(lines[0], lines[1])
            l0l2_angle = angle_between_lines(lines[0], lines[2])
            l1l2_angle = angle_between_lines(lines[1], lines[2])
            if min(l0l1_angle, l0l2_angle, l1l2_angle) == l1l2_angle:
                lines.pop(0)
            elif min(l0l1_angle, l0l2_angle, l1l2_angle) == l0l2_angle:
                lines.pop(1)
            else:
                lines.pop(2)
        # Get slope.
        s1 = lines[0]
        s2 = lines[1]
        # Average the lines to get better approximation of actual line.
        s1_angle = (180 * math.atan(s1[0]) / math.pi) % 180
        s2_angle = (180 * math.atan(s2[0]) / math.pi) % 180
        # Ensure that angles are within 90 degrees of each other so we 
        # always average the acute angle between lines.
        if abs(s1_angle - s2_angle) > 90:
            s1_angle = s1_angle - 180
        # Negate since y starts at top of image.
        avg_angle = -(s1_angle + s2_angle) / 2
        avg_slope = math.tan(math.pi * avg_angle / 180)

        angle1 = avg_angle
        if angle1 > 0:
            angle2 = angle1 - 180
        else:
            angle2 = angle1 + 180

        finalLines = [angle1, angle2]

        M = cv2.moments(255 - thresh_image)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centerPoint = (cX, cY)

        return finalLines, centerPoint
    else:
        # Combine the 4 lines into 2 most different lines.
        finalLines = []
        # Array to hold the 4 lines, organized by heading (2 lines 
        # per heading, upper and lower).
        linesToFindCenter = []
        for i in range(2):
            # Get slope.
            s1 = lines[0]
            lines.remove(s1)
            # Get line with least angle from s1.
            s2 = min(lines, key=lambda x: angle_between_lines(s1, x))
            lines.remove(s2)
            # Average the lines to get better approximation of actual line.
            s1_angle = (180 * math.atan(s1[0]) / math.pi) % 180
            s2_angle = (180 * math.atan(s2[0]) / math.pi) % 180
            # Ensure that angles are within 90 degrees of each other so we 
            # always average the acute angle between lines.
            if abs(s1_angle - s2_angle) > 90:
                s1_angle = s1_angle - 180
            avg_angle = (s1_angle + s2_angle) / 2
            avg_slope = math.tan(math.pi * avg_angle / 180)
            finalLines.append(avg_slope)
            # Get the upper and lower lines by comparing their y-intercept values.
            upper_line = min((s1, s2), key=lambda x: x[1])
            lower_line = max((s1, s2), key=lambda x: x[1])
            # Save upper and lower line to the edge lines array.
            linesToFindCenter.append((upper_line, lower_line))

        # Get the center point of the lane marker using the rectangle 
        # defined by the 4 lines on the lane markers edges.
        centerPoint = get_rectangle_center_point(linesToFindCenter)
        if (
            centerPoint == None
            or centerPoint[0] < 0
            or centerPoint[1] < 0
            or centerPoint[0] >= thresh_image.shape[1]
            or centerPoint[1] >= thresh_image.shape[0]
        ):
            return None, None
        avgs = []
        for slope in finalLines:
            dilation_kernel = np.ones((5, 5), np.uint8)
            inverted_thresh_image = cv2.bitwise_not(thresh_image)
            while (
                not inverted_thresh_image[int(centerPoint[1])][int(centerPoint[0])] == 255
            ):
                inverted_thresh_image = cv2.dilate(
                    inverted_thresh_image, dilation_kernel, iterations=1
                )
            thresh_image = cv2.bitwise_not(inverted_thresh_image)
            negStepsWithLM = get_step_with_LM(thresh_image, slope, -1, centerPoint)
            posStepsWithLM = get_step_with_LM(thresh_image, slope, 1, centerPoint)
            avgs.append([slope, posStepsWithLM, 1])
            avgs.append([slope, negStepsWithLM, -1])
        s1 = max(avgs, key=lambda x: x[1])
        avgs.remove(s1)
        s2 = max(avgs, key=lambda x: x[1])
        # negated because y axis is 0 at top of frame
        angle1 = -180 * (math.atan(s1[0]) / math.pi)
        angle2 = -180 * (math.atan(s2[0]) / math.pi)
        if s1[2] < 0:  # bs(angle) should be above 90.
            if abs(angle1) < 90:
                if angle1 < 0:
                    angle1 += 180
                else:
                    angle1 -= 180
        else:  # abs(angle) should be below 90.
            if abs(angle1) > 90:
                if angle1 < 0:
                    angle1 += 180
                else:
                    angle1 -= 180
        if s2[2] < 0:  # abs(angle) should be above 90.
            if abs(angle2) < 90:
                if angle2 < 0:
                    angle2 += 180
                else:
                    angle2 -= 180
        else:  # abs(angle) should be below 90.
            if abs(angle2) > 90:
                if angle2 < 0:
                    angle2 += 180
                else:
                    angle2 -= 180
        finalLines = [angle1, angle2]
        return finalLines, centerPoint


def get_step_with_LM(image, slope, direction, center_point, lm_color=0):
    x, y = center_point
    num_steps = 50
    if abs(slope) > 1: # Slopes that change faster in y than x.
        step_y = image.shape[0] / (2 * num_steps)
        if slope < 0:
            step_y *= -1.0 # Step_y must always have same sign as the slope.
        step_x = abs(step_y / slope) # Step_x must always be positive.
    else:
        step_x = image.shape[1] / (2 * num_steps)
        step_y = slope * step_x

    stepsWithLM = 0
    while x < image.shape[1] and y < image.shape[0] and x > 0 and y > 0:
        if image[int(y)][int(x)] == lm_color:
            stepsWithLM += 1
        x += step_x * direction
        y += step_y * direction
    return stepsWithLM


def visualize_lane_marker(image, debug=True):
    # Crop image to lane marker.
    line_thickness = 1  # In pixels.
    line_length = int(
        0.25 * image.shape[1]
    )  # In pixels, line will be 1/4 of bounding box width.
    # Measure headings from lane marker.
    headings, center_point = measure_headings(image, debug)
    if None in (headings, center_point):
        return image
    for angle in headings:
        # Get angle, line start and line end from heading slope.
        slope = math.tan((angle / -180) * math.pi)
        line_x_length = math.sqrt((line_length**2) / (1 + slope**2))
        if abs(angle) > 90:  # Heading goes into negative x.
            line_end = (
                int(center_point[0] - line_x_length),
                int(center_point[1] - slope * line_x_length),
            )  # (x,y).
        else:  # Heading goes into positive x.
            line_end = (
                int(center_point[0] + line_x_length),
                int(center_point[1] + slope * line_x_length),
            )  # (x,y).
        # Draw line on original image.
        cv2.line(image, center_point, line_end, (255, 0, 0), line_thickness)
        # Add text with measured angle of line at the end of the line.
        cv2.putText(
            image,
            text=str(angle) + " deg.",
            org=line_end,
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.4,
            color=(255, 0, 0),
            lineType=cv2.LINE_AA,
        )
    cv2.circle(image, center_point, radius=5, color=(255, 0, 0), thickness=-1)
    return image

def measure_lane_marker(image, bbox, debug_image):
    # Crop image to lane marker.
    cropped_image = crop_to_bbox(image, bbox)
    line_thickness = 2  # in pixels
    line_length = 0.25 * min(
        bbox[2], bbox[3]
    )  # Line will be size of shortest bounding box side.
    # Measure headings from lane marker.
    cropped_image_to_pub = bridge.cv2_to_imgmsg(cropped_image, "bgr8")
    if not TESTING:
        pub_cropped_image.publish(cropped_image_to_pub)
    headings, center_point = measure_headings(
        cropped_image, debug_image = crop_to_bbox(debug_image, bbox, copy=False)
    )
    if None in (headings, center_point):
        return (None, None), (None, None), debug_image
    center_point_x = center_point[0] + bbox[0] - bbox[2] / 2
    center_point_y = center_point[1] + bbox[1] - bbox[3] / 2
    center_point = (int(center_point_x), int(center_point_y))
    for angle in headings:
        # Get angle, line start and line end from heading slope.
        slope = math.tan((angle / -180) * math.pi)
        line_x_length = math.sqrt((line_length**2) / (1 + slope**2))
        if abs(angle) > 90:  # Heading goes into negative x.
            line_end = (
                int(center_point[0] - line_x_length),
                int(center_point[1] - slope * line_x_length),
            )  # (x,y).
        else:  # Heading goes into positive x.
            line_end = (
                int(center_point[0] + line_x_length),
                int(center_point[1] + slope * line_x_length),
            )  # (x,y).
        # Draw line on image.
        cv2.line(debug_image, center_point, line_end, HEADING_COLOR, line_thickness)
        # Add text with measured angle of line at the end of the line.
        cv2.putText(
            debug_image,
            text=str(angle) + " deg.",
            org=line_end,
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.4,
            color=HEADING_COLOR,
            lineType=cv2.LINE_AA,
        )
    cv2.circle(debug_image, center_point, radius=5, color=HEADING_COLOR, thickness=-1)
    return headings, center_point, debug_image


if __name__ == "__main__":
    # This is only executed if this script is executed directly.
    # Used for testing purposes.
    rospy.init_node("lane_marker_measure")