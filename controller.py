import cv2
import numpy as np
import socket
from config import CONTROL_IP, CONTROL_PORT

sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sk.settimeout(3000)


def set_control_ip():
    ip = socket.gethostbyname(socket.gethostname())
    control_msg = "SET_CONTROL_IP {}".format(ip).encode('ascii')
    sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))


def send_control(left_motor_speed, right_motor_speed):
    """Convert steering and throttle signals to a suitable format and send them to ESP32 bot"""
    control_msg = "CONTROL_WHEEL {} {}".format(
        left_motor_speed, right_motor_speed).encode('ascii')
    sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))


def calculate_control_signal(left_point, right_point, im_center):
    """Calculate control signal"""

    '''if left_point == -1 or right_point == -1:
        left_motor_speed = right_motor_speed = 100
        return left_motor_speed, right_motor_speed'''

    # Calculate difference between car center point and image center point
    center_point = (right_point + left_point) // 2
    center_diff = center_point - im_center

    # Calculate steering angle from center point difference
    steering = -float(center_diff * (0.0025 / 2))
    steering = min(1, max(-1, steering))
    throttle = 0.83

    # From steering, calculate left/right motor speed
    left_motor_speed = 0
    right_motor_speed = 0

    if steering > 0:
        left_motor_speed = throttle * (1 - steering)
        right_motor_speed = throttle
    else:
        left_motor_speed = throttle
        right_motor_speed = throttle * (1 + steering)

    left_motor_speed = int(left_motor_speed * 100)
    right_motor_speed = int(right_motor_speed * 100)
    if abs(left_motor_speed - right_motor_speed) <= 5:
        right_motor_speed = 83
        left_motor_speed = 83
    elif 5 < left_motor_speed - right_motor_speed <= 10:
        right_motor_speed = 83
        left_motor_speed = 85
    elif 5 < right_motor_speed - left_motor_speed <= 10:
        right_motor_speed = 85
        left_motor_speed = 83
    elif 10 < left_motor_speed - right_motor_speed <= 20:
        right_motor_speed = 83
        left_motor_speed = 86
    elif 10 < right_motor_speed - left_motor_speed <= 20:
        right_motor_speed = 86
        left_motor_speed = 83
    elif 20 < left_motor_speed - right_motor_speed <= 30:
        right_motor_speed = 83
        left_motor_speed = 87
    elif 20 < right_motor_speed - left_motor_speed <= 30:
        right_motor_speed = 87
        left_motor_speed = 83
    elif 30 < left_motor_speed - right_motor_speed <= 40:
        right_motor_speed = 83
        left_motor_speed = 88
    elif 30 < right_motor_speed - left_motor_speed <= 40:
        right_motor_speed = 88
        left_motor_speed = 83


    return left_motor_speed, right_motor_speed


def grayscale(img):
    """Convert image to grayscale"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def canny(img, low_threshold, high_threshold):
    """Apply Canny edge detection"""
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
    """Apply a Gaussian blur"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def birdview_transform(img):
    """Get birdview image"""
    IMAGE_H = 296
    IMAGE_W = 400

    src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [
        0, IMAGE_H // 3], [IMAGE_W, IMAGE_H // 3]])
    dst = np.float32([[110, IMAGE_H], [290, IMAGE_H],
                      [-20, 0], [IMAGE_W, 0]])
    M = cv2.getPerspectiveTransform(src, dst)  # The transformation matrix
    warped_img = cv2.warpPerspective(
        img, M, (IMAGE_W, IMAGE_H))  # Image warping
    return warped_img


def preprocess(img):
    """Preprocess image to get a birdview image of lane lines"""

    img = grayscale(img)
    img = gaussian_blur(img, 3)
    img = canny(img, 100, 200)
    cv2.imshow("Canny", img)
    cv2.waitKey(2)
    img = birdview_transform(img)

    return img


def find_lane_lines(image, draw=False):
    """Find lane lines from color image"""

    image = preprocess(image)

    im_height, im_width = image.shape[:2]

    if draw:
        viz_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # Interested line to determine lane center
    interested_line_y = int(im_height * 0.7)
    interested_line_y_up = int(im_height * 0.5)
    if draw:
        cv2.line(viz_img, (0, interested_line_y),
                 (im_width, interested_line_y), (0, 0, 255), 2)
        cv2.line(viz_img, (0, interested_line_y_up),
                 (im_width, interested_line_y_up), (0, 0, 255), 2)
    interested_line = image[interested_line_y, :]
    interested_line_up = image[interested_line_y_up, :]

    # Determine left point and right point
    left_point = -1
    right_point = -1
    left_point_up = -1
    right_point_up = -1
    lane_width = 212
    right_points_before = [-1, -1, -1, -1, -1]
    right_points_up_before = [-1, -1, -1, -1, -1]
    center = (im_width // 2) + 25

    for x in range(center, 0, -1):
        if interested_line[x] > 0:
            left_point = x
            break
    for x in range(center + 1, im_width):

        right_points_before.append(right_point)
        right_points_before.pop(0)
        if interested_line[x] > 0:
            right_point = x
            break

    for x in range(center, 0, -1):
        if interested_line_up[x] > 0:
            left_point_up = x
            break
    for x in range(center + 1, im_width):

        right_points_up_before.append(right_point_up)
        right_points_up_before.pop(0)
        if interested_line_up[x] > 0:
            right_point_up = x
            break

    if right_points_before[4] != -1:
        gap_right = abs(right_point - right_points_before[4])
        if gap_right > 50:
            right_point = sum(right_points_before) // 5


    if right_points_up_before[4] != -1:
        gap_right_up = abs(right_point_up - right_points_up_before[4])
        if gap_right_up > 50:
            right_point_up = sum(right_points_up_before) // 5

    # Predict occluded points
    if left_point != -1 and right_point == -1:
        right_point = left_point + lane_width
    if left_point == -1 and right_point != -1:
        left_point = right_point - lane_width

    if left_point_up != -1 and right_point_up == -1:
        right_point_up = left_point_up + lane_width
    if left_point_up == -1 and right_point_up != -1:
        left_point_up = right_point_up - lane_width

    if abs(left_point - right_point) < 50:
        right_point = sum(right_points_before) // 5
        left_point = right_point - lane_width

    if abs(left_point_up - right_point_up) < 50:
        right_point_up = sum(right_points_up_before) // 5
        left_point_up = right_point_up - lane_width



    if draw:
        if left_point != -1:
            viz_img = cv2.circle(
                viz_img, (left_point, interested_line_y), 7, (255, 255, 0), -1)
        if right_point != -1:
            viz_img = cv2.circle(
                viz_img, (right_point, interested_line_y), 7, (0, 255, 0), -1)
        if left_point_up != -1:
            viz_img = cv2.circle(
                viz_img, (left_point_up, interested_line_y_up), 7, (255, 255, 0), -1)
        if right_point_up != -1:
            viz_img = cv2.circle(
                viz_img, (right_point_up, interested_line_y_up), 7, (0, 255, 0), -1)

    if draw:
        return left_point, right_point, center, left_point_up, right_point_up, viz_img
    else:
        return left_point, right_point, center, left_point_up, right_point_up
