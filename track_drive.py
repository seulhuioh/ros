#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

image = np.empty(shape=[0])
ranges = None
motor = None
motor_msg = XycarMotor()
Fix_Speed = 10
new_angle = 0
new_speed = Fix_Speed
bridge = CvBridge()

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')

def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def lidar_callback(data):
    global ranges
    ranges = data.ranges[0:360]

def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)

def detect_green_light(img):
    if img is None or img.size == 0:
        print("Image is empty or not initialized.")
        return False

    h, w = img.shape[:2]
    y = 100
    cv2.circle(img, (w // 2, y), 5, (255, 255, 255), -1)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    center_pixel = hsv[y, w // 2]
    bgr_pixel = img[y, w // 2]
    print(f"Center HSV: {center_pixel} | BGR: {bgr_pixel}")

    green_mask = cv2.inRange(hsv, (50, 100, 100), (90, 255, 255))
    green_area = cv2.countNonZero(green_mask)
    print(f"Green pixel count: {green_area}")
    return green_area > 1000

def process_lane(img):
    h, w = img.shape[:2]
    roi = img[h//2:h, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 흰색 실선 기준 차선 인식
    white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
    M = cv2.moments(white_mask)
    cx = w // 2

    if M['m00'] != 0:
        cx_white = int(M['m10'] / M['m00'])
        cx = cx_white - 30  # 흰 실선보다 왼쪽으로 약간 이동한 위치를 따라감
        cv2.circle(roi, (cx_white, h // 4), 5, (255, 255, 255), -1)
        cv2.circle(roi, (cx, h // 4), 5, (0, 255, 0), -1)

    error = cx - (w // 2)
    angle = -error / 3.0
    return angle

def start():
    global motor, image, ranges
    print("Start program --------------")

    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("/scan", LaserScan)
    print("Lidar Ready ----------")

    plt.ion()
    plt.show()
    print("Lidar Visualizer Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    start_flag = False

    while not rospy.is_shutdown():
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("original", image)
        cv2.imshow("gray", gray)

        if ranges is not None:
            angles = np.linspace(0, 2*np.pi, len(ranges)) + np.pi/2
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            lidar_points.set_data(x, y)
            fig.canvas.draw_idle()
            plt.pause(0.01)

        if not start_flag:
            start_flag = detect_green_light(image)
            speed = 0
            angle = 0.0
            print("Waiting for green light...")
        else:
            speed = Fix_Speed
            angle = process_lane(image)
            print("Green light detected. Driving...")

        drive(angle=angle, speed=speed)
        time.sleep(0.1)
        cv2.waitKey(1)

if __name__ == '__main__':
    start()