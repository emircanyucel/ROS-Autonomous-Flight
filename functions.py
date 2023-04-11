"""
---These codes are created by Emircan Yücel---

In this script, we crated some functions with ROS, Dronekit and Pymavlink. This script include image processing algorithm,
lidar data extraction, and some necessary functions for autonomous flight.

You can reach my tutorial content, which I have prepared for a detailed explanation of this, and similar content,
where I share my experience and knowledge, from the link below.
https://medium.com/@emircanyucel27
To contact:
https://www.linkedin.com/in/emircan-y%C3%BCcel-267475246
"""

#! /usr/bin/env python3
# -*- coding: UTF-8 -*-

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError


class Drone:

    def __init__(self):
        self.connection_string = "127.0.0.1:14551"
        self.vehicle = connect(self.connection_string, wait_ready=['mode', 'airspeed'])  # wait_ready = True
        self.vehicle.wait_ready('autopilot_version')
        self.next_waypoint = 0
        self.kamera_degisken = 0
        self.process_lidar_data = True
        self.komut = None
        self.lidar_kontrol = True
        self.k = 0
        self.aux_channel1 = 9
        self.aux_channel2 = 10

    def modeAyarla(self, mode):
        self.vehicle.mode = VehicleMode(mode)
        print("Mode has been changed as a {}".format(mode))

    def arm_and_takeoff(self, aTargetAltitude):
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    def set_servo(self, servo_number, pwm_value):
        pwm_value_int = int(pwm_value)
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_number,
            pwm_value_int,
            0, 0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg)

    def move(self, x, y, z):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b0000011111111000,  # type_mask (only positions enabled)
            x, y, z,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to drone
        self.vehicle.send_mavlink(msg)

    def moving(self, ax, ay):
        if self.vehicle.mode != 'GUIDED' or 'POSHOLD':
            self.vehicle.mode = VehicleMode("GUIDED")
        elif self.vehicle.mode == 'GUIDED':
            if ax < 0 and ay > 0:
                if ax < -50 and ay > 40:
                    self.move(0.5, -0.5, 0.5)
                elif ax < -50 and 40 > ay > 0:
                    self.move(0, -0.5, 0.5)
                elif -50 < ax < 0 and ay > 40:
                    self.move(0.5, 0, 0.5)
                elif -50 < ax < 0 and 0 < ay < 40:
                    if self.vehicle.location.global_relative_frame.alt <= 1 * 0.95:
                        print(self.vehicle.location.global_relative_frame.alt)
                        self.vehicle.mode = VehicleMode("LAND")


                    else:
                        self.move(0, 0, 0.5)
            # print("İleri ve Sol")
            elif ax > 0 and ay > 0:
                if ax > 50 and ay > 40:
                    self.move(0.5, 0.5, 0.5)
                elif ax > 50 and 40 > ay > 0:
                    self.move(0, 0.5, 0.5)
                elif 50 > ax > 0 and ay > 40:
                    self.move(0.5, 0, 0.5)
                elif 50 > ax > 0 and 0 < ay < 40:
                    if self.vehicle.location.global_relative_frame.alt <= 1 * 0.95:
                        print(self.vehicle.location.global_relative_frame.alt)
                        self.vehicle.mode = VehicleMode("LAND")

                    else:
                        self.move(0, 0, 0.5)
            # print("Sağ ve İleri")
            elif ax < 0 and ay < 0:
                if ax < -50 and ay < -40:
                    self.move(-0.5, -0.5, 0.5)
                elif ax < -50 and -40 < ay < 0:
                    self.move(0, -0.5, 0.5)
                elif -50 < ax < 0 and ay < -40:
                    self.move(0, -0.5, 0.5)
                elif -50 < ax < 0 and 0 > ay > -40:
                    if self.vehicle.location.global_relative_frame.alt <= 1 * 0.95:
                        print(self.vehicle.location.global_relative_frame.alt)
                        self.vehicle.mode = VehicleMode("LAND")

                    else:
                        self.move(0, 0, 0.5)
            # print("Geri ve Sol")
            elif ax > 0 and ay < 0:
                if ax > 50 and ay < -40:
                    self.move(-0.5, 0.5, 0.5)
                elif ax > 50 and -40 < ay < 0:
                    self.move(0, 0.5, 0.5)
                elif 50 > ax > 0 and ay < -40:
                    self.move(-0.5, 0, 0.5)
                elif 50 > ax > 0 and 0 > ay > -40:
                    if self.vehicle.location.global_relative_frame.alt <= 1 * 0.95:
                        print(self.vehicle.location.global_relative_frame.alt)
                        self.vehicle.mode = VehicleMode("LAND")

                    else:
                        self.move(0, 0, 0.5)
            # print("İleri")
            else:
                if self.vehicle.location.global_relative_frame.alt <= 1 * 0.95:
                    print(self.vehicle.location.global_relative_frame.alt)
                    self.vehicle.mode = VehicleMode("LAND")


    def movingH(self, ax, ay):
        if self.vehicle.mode != 'GUIDED' or 'POSHOLD':
            self.vehicle.mode = VehicleMode("GUIDED")
        elif self.vehicle.mode == 'GUIDED':
            if ax < 0 and ay > 0:
                if ax < -50 and ay > 40:
                    self.move(0.5, -0.5, 0)
                elif ax < -50 and 40 > ay > 0:
                    self.move(0, -0.5, 0)
                elif -50 < ax < 0 and ay > 40:
                    self.move(0.5, 0, 0)
                elif -50 < ax < 0 and 0 < ay < 40:

                    self.set_servo(self.aux_channel1, 1500)
            # print("İleri ve Sol")
            elif ax > 0 and ay > 0:
                if ax > 50 and ay > 40:
                    self.move(0.5, 0.5, 0)
                elif ax > 50 and 40 > ay > 0:
                    self.move(0, 0.5, 0)
                elif 50 > ax > 0 and ay > 40:
                    self.move(0.5, 0, 0)
                elif 50 > ax > 0 and 0 < ay < 40:

                    self.set_servo(self.aux_channel1, 1500)
            # print("Sağ ve İleri")
            elif ax < 0 and ay < 0:
                if ax < -50 and ay < -40:
                    self.move(-0.5, -0.5, 0)
                elif ax < -50 and -40 < ay < 0:
                    self.move(0, -0.5, 0)
                elif -50 < ax < 0 and ay < -40:
                    self.move(0, -0.5, 0)
                elif -50 < ax < 0 and 0 > ay > -40:

                    self.set_servo(self.aux_channel1, 1500)
            # print("Geri ve Sol")
            elif ax > 0 and ay < 0:
                if ax > 50 and ay < -40:
                    self.move(-0.5, 0.5, 0)
                elif ax > 50 and -40 < ay < 0:
                    self.move(0, 0.5, 0)
                elif 50 > ax > 0 and ay < -40:
                    self.move(-0.5, 0, 0)
                elif 50 > ax > 0 and 0 > ay > -40:

                    self.set_servo(self.aux_channel1, 1500)
            # print("İleri")
            else:
                if 50 > ax > 0 and 0 > ay > -40:

                    self.set_servo(self.aux_channel1, 1500)
                else:
                    print("Mode: {}".format(self.vehicle.mode))


    def gorevListesi(self):
        komut = self.vehicle.commands
        print("Waypoints has been taken.")
        return komut

    def airSpeed(self, deger):
        self.vehicle.airspeed = deger
        print("Airspeed {} degerine ayarlandi.".format(deger))

    def groundSpeed(self, deger):
        self.vehicle.groundspeed = deger
        print("Groundspeed {} degerine ayarlandi.".format(deger))

    def ImageConverter(self):
        self.image_pub = rospy.Publisher("image_converter/output_video", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.image_cb)
        cv2.namedWindow("Image")

    def image_cb(self, msg):
        global cv_image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        src = cv_image

        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)

        low_blue = np.array([0, 150, 0])
        high_blue = np.array([5, 255, 255])

        mask = cv2.inRange(hsv, low_blue, high_blue)

        _, cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        def getLargestRedContour(img, contours):
            sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)

            for cnt in sortedContours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    M = cv2.moments(cnt)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
                    cv2.circle(img, (cx, cy), 3, (0, 0, 0), -1)
                    cv2.putText(img, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    # print("x: {} y: {}".format(((cx + 1) - 320), (-cy + 240)))
                    cx = (cx + 1) - 320
                    cy = -cy + 240
                    print("X: {}   Y:{}".format(cx, cy))
                    if cx or cy != 0:
                        self.movingH(cx, cy)
                    else:
                        print("X: {}   Y:{}".format(cx, cy))

        getLargestRedContour(src, cnts)

        cv2.imshow("source", src)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(hsv, "8UC3"))
        except CvBridgeError as e:
            print(e)

    def ImageConverterKM(self):
        self.image_pub = rospy.Publisher("image_converter/output_video", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.image_cb2)
        cv2.namedWindow("Goruntu")

    def image_cb2(self, msg):
        global cv_image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        src = cv_image

        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)

        def getLargestRedOrBlueContour(img, red_contours, blue_contours):

            sortedRedContours = sorted(red_contours, key=cv2.contourArea, reverse=True)
            sortedBlueContours = sorted(blue_contours, key=cv2.contourArea, reverse=True)

            if len(sortedRedContours) == 0 or len(sortedBlueContours) == 0:
                print("Renk Aranıyor.")
            else:

                largestRedContour = sortedRedContours[0]
                largestBlueContour = sortedBlueContours[0]

                red_area = cv2.contourArea(largestRedContour)
                blue_area = cv2.contourArea(largestBlueContour)

                if red_area > 15000:
                    x, y, w, h = cv2.boundingRect(largestBlueContour)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    M = cv2.moments(largestBlueContour)
                    cx = int(M['m10'] / M['m00'])
                    x = (cx + 1) - 320
                    cy = int(M['m01'] / M['m00'])
                    y = -cy + 240
                    cv2.drawContours(img, [largestBlueContour], -1, (0, 255, 0), 2)
                    cv2.circle(img, (cx, cy), 3, (0, 0, 0), -1)
                    cv2.putText(img, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    if x or y != 0:
                        self.moving(x, y)
                    else:
                        print("X: {}   Y:{}".format(x, y))

                elif red_area > 500:
                    x, y, w, h = cv2.boundingRect(largestRedContour)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    M = cv2.moments(largestRedContour)
                    cx = int(M['m10'] / M['m00'])
                    x = (cx + 1) - 320
                    cy = int(M['m01'] / M['m00'])
                    y = -cy + 240
                    cv2.drawContours(img, [largestRedContour], -1, (0, 255, 0), 2)
                    cv2.circle(img, (cx, cy), 3, (0, 0, 0), -1)
                    cv2.putText(img, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    if x or y != 0:
                        self.moving(x, y)
                    else:
                        print("X: {}   Y:{}".format(x, y))

        def kirmizi():
            low_red = np.array([0, 150, 0])
            high_red = np.array([5, 255, 255])

            mask_red = cv2.inRange(hsv, low_red, high_red)

            _, cnts_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            return cnts_red

        def mavi():
            low_blue = np.array([100, 150, 0])
            high_blue = np.array([140, 255, 255])
            mask_blue = cv2.inRange(hsv, low_blue, high_blue)

            _, cnts_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            return cnts_blue

        kirmizi = kirmizi()
        mavi = mavi()

        getLargestRedOrBlueContour(src, kirmizi, mavi)

        cv2.imshow("source", src)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(hsv, "8UC3"))
        except CvBridgeError as e:
            print(e)

    def mainIC(self):
        rospy.init_node("image_converter", anonymous=True)
        self.ImageConverter()
        while not rospy.is_shutdown():
            rospy.spin()

    def mainICKM(self):
        rospy.init_node("image_converter", anonymous=True)
        self.ImageConverterKM()
        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self, data):

        filtered_ranges = [x for x in data.ranges if x != float('inf')]
        if filtered_ranges:
            min_distance = min(filtered_ranges)
            if float(1) < min_distance < float(5):
                print(min_distance)
                self.modeAyarla("GUIDED")
                rospy.signal_shutdown("rospy.spin kapatildi")
            else:
                print(min_distance)
                print("engel yok")
        else:
            print("Aralık inf")

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(name="/spur/laser/scan", data_class=LaserScan, queue_size=1, callback=self.laser_callback)
        while not rospy.is_shutdown():
            rospy.spin()
        print("Lidar Listener Dongusunden Cikildi")

    def download_mission(self):
        print(" Download mission from vehicle")
        missionlist = []
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        for cmd in cmds:
            missionlist.append(cmd)
        return missionlist

    def add_last_waypoint_to_mission(self, lat, lon, alt):
        self.download_mission()
        cmds = self.vehicle.commands

        missionList = []

        for wp in cmds:
            missionList.append(wp)

        wp_last = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                          0,
                          0, 0, 0, 0, 0, lat, lon, alt)

        missionList.append(wp_last)
        cmds.clear()

        for wp in missionList:
            cmds.add(wp)

        cmds.upload()

        return cmds.count

    def gorev_ekle(self):
        self.komut = self.vehicle.commands

        self.komut.clear()
        time.sleep(1)

        # TAKEOFF
        self.komut.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                    0, 0,
                    0, 0, 0, 0, 10))

        # WAYPOINT
        self.komut.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0,
                    0, 0, 0, -35.3639803, 149.1652054, 10))

