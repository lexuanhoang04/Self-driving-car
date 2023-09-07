import urllib.request
import _thread
import cv2
import numpy as np
import time

from controller5 import calculate_control_signal, send_control, set_control_ip, find_lane_lines
from config import CAM_URL

stream = urllib.request.urlopen(CAM_URL)
bytes = bytes()


# Set control ip continuously to receive sensor params
def set_control_ip():
    time.sleep(2)
    set_control_ip()


_thread.start_new_thread(set_control_ip, ())

while True:

    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')

    if a != -1 and b != -1:
        image = bytes[a:b + 2]
        bytes = bytes[b + 2:]

        try:
            # Decode image
            image = cv2.imdecode(np.fromstring(image, dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow('Image', image)
        except:
            if cv2.waitKey(1) == 27:
                exit(0)
            continue

        left_point, right_point, im_center, left_point_up, right_point_up, draw = find_lane_lines(
            image, draw=True)
        cv2.imshow("Lane lines", draw)
        cv2.waitKey(2)

        # Calculate speed and steering angle

        left_motor_speed, right_motor_speed = calculate_control_signal(left_point, right_point, im_center)
        '''if left_point_up - left_point > 60:
            left_motor_speed = 83
            right_motor_speed = 83
            count = 1
            while count < 900:
                if count % 100 == 0:
                    left_motor_speed += 3
                    send_control(left_motor_speed, right_motor_speed)
                count += 1'''

        send_control(left_motor_speed, right_motor_speed)
        #send_control(0, 0)

        print(left_point, right_point, left_motor_speed, right_motor_speed, left_point_up, right_point_up)

        if cv2.waitKey(1) == 27:
            send_control(0, 0)
            exit(0)
