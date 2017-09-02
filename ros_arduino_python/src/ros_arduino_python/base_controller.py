#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller

    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os
import tf

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame):
        self.arduino = arduino
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False

        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "")
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "")
        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)
        pid_params['Kp'] = rospy.get_param("~Kp", 20)
        pid_params['Kd'] = rospy.get_param("~Kd", 12)
        pid_params['Ki'] = rospy.get_param("~Ki", 0)
        pid_params['Ko'] = rospy.get_param("~Ko", 50)

        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)

        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)

        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)

        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate

        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        now = rospy.Time.now()
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta



	# Internal data
        self.enc_wheel1 = None            # encoder readings
	self.enc_wheel2 = None
	self.enc_wheel3 = None
	self.x = 0			  # position in xy plane
	self.y = 0
	self.th = 0
	self.v_wheel1 = 0
        self.v_wheel2 = 0
	self.v_wheel3 = 0
	self.v_des_wheel1 = 0
	self.v_des_wheel2 = 0
	self.v_des_wheel3 = 0
	self.last_cmd_vel = now


         # Internal data
#        self.enc_left = None            # encoder readings
#        self.enc_right = None
#        self.x = 0                      # position in xy plane
#        self.y = 0
#        self.th = 0                     # rotation in radians
#        self.v_left = 0
#        self.v_right = 0
#        self.v_des_left = 0             # cmd_vel setpoint
#        self.v_des_right = 0
#        self.last_cmd_vel = now

        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

        # Clear any old odometry info
        self.arduino.reset_encoders()

        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()

        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True

        if missing_params:
            os._exit(1)

        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']

        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']

        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            # Read the encoders
            try:
                wheel1_enc, wheel2_enc, wheel3_enc = self.arduino.get_encoder_counts()
#			     left_enc, right_enc = self.arduino.get_encoder_counts()
            except:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return

            dt = now - self.then
            self.then = now
            dt = dt.to_sec()

            # Calculate odometry
			# Calculate wheelspeed by encoder
            if self.enc_wheel1 == None:
                dwheel1 = 0
                dwheel2 = 0
                dwheel3 = 0
            else:
                dwheel1= (wheel1_enc - self.enc_wheel1) / self.ticks_per_meter
                dwheel2 = (wheel2_enc - self.enc_wheel2) / self.ticks_per_meter
		dwheel3 = (wheel3_enc - self.enc_wheel3) / self.ticks_per_meter

            self.enc_wheel1 = wheel1_enc
            self.enc_wheel2 = wheel2_enc
	    self.enc_wheel3 = wheel3_enc

#	    if self.enc_left == None:
#                dright = 0
#                dleft = 0
#            else:
#                dright = (right_enc - self.enc_right) / self.ticks_per_meter
#                dleft = (left_enc - self.enc_left) / self.ticks_per_meter

#            self.enc_right = right_enc
#            self.enc_left = left_enc

            df_vx = (2 * dwheel2 - dwheel1 - dwheel3) / 3
	    df_vy = (dwheel3 - dwheel1) * 0.57735
	    df_vth = (dwheel1 + dwheel2 + dwheel3) / (3 * self.wheel_track)

	    delta_x = (df_vx * cos(self.th) - df_vy * sin(self.th)) * dt
            delta_y = (df_vx * sin(self.th) + df_vy * cos(self.th)) * dt
            delta_th = df_vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

	    self.x *= 9.0
            self.y *= 9.0
            self.th *= 9.0

#	     dxy_ave = (dright + dleft) / 2.0
#            dth = (dright - dleft) / self.wheel_track
#            vxy = dxy_ave / dt
#            vth = dth / dt
#
#            if (dxy_ave != 0):
#                dx = cos(dth) * dxy_ave
#                dy = -sin(dth) * dxy_ave
#                self.x += (cos(self.th) * dx - sin(self.th) * dy)
#                self.y += (sin(self.th) * dx + cos(self.th) * dy)
#
#            if (dth != 0):
#                self.th += dth


	    quaternion = tf.transformations.quaternion_from_euler(0, 0, self.th)

#            quaternion = Quaternion()
#            quaternion.x = 0.0
#            quaternion.y = 0.0
#            quaternion.z = sin(self.th / 2.0)
#            quaternion.w = cos(self.th / 2.0)

            # Create the odometry transform frame broadcaster.
	    self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                quaternion,
                rospy.Time.now(),
                self.base_frame,
                "odom"
                )

#            self.odomBroadcaster.sendTransform(
#                (self.x, self.y, 0),
#                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
#                rospy.Time.now(),
#                self.base_frame,
#                "odom"
#                )

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation.x = quaternion[0]
	    odom.pose.pose.orientation.y = quaternion[1]
	    odom.pose.pose.orientation.z = quaternion[2]
	    odom.pose.pose.orientation.w = quaternion[3]
            odom.twist.twist.linear.x = df_vx
            odom.twist.twist.linear.y = df_vy
            odom.twist.twist.angular.z = df_vth

            self.odomPub.publish(odom)

            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
	 	        self.v_des_wheel1 = 0
                        self.v_des_wheel2 = 0
		        self.v_des_wheel3 = 0
#                self.v_des_left = 0
#                self.v_des_right = 0

            if self.v_wheel1 < self.v_des_wheel1:
                self.v_wheel1 += self.max_accel
                if self.v_wheel1 > self.v_des_wheel1:
                    self.v_wheel1 = self.v_des_wheel1
            else:
                self.v_wheel1 -= self.max_accel
                if self.v_wheel1 < self.v_des_wheel1:
                    self.v_wheel1 = self.v_des_wheel1

            if self.v_wheel2 < self.v_des_wheel2:
                self.v_wheel2 += self.max_accel
                if self.v_wheel2 > self.v_des_wheel2:
                    self.v_wheel2 = self.v_des_wheel2
            else:
                self.v_wheel2 -= self.max_accel
                if self.v_wheel2 < self.v_des_wheel2:
                    self.v_wheel2 = self.v_des_wheel2

	    if self.v_wheel3 < self.v_des_wheel3:
                self.v_wheel3 += self.max_accel
                if self.v_wheel3 > self.v_des_wheel3:
                    self.v_wheel3 = self.v_des_wheel3
            else:
                self.v_wheel3 -= self.max_accel
                if self.v_wheel3 < self.v_des_wheel3:
                    self.v_wheel3 = self.v_des_wheel3

            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
		self.arduino.drive(self.v_wheel1, self.v_wheel2, self.v_wheel3)
#                self.arduino.drive(self.v_left, self.v_right)

            self.t_next = now + self.t_delta

    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0, 0)
#        self.arduino.drive(0, 0)

    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()

	th = req.angular.z       # rad/s
	x = req.linear.x         # m/s
	y = req.linear.y

        if x == 0 and y == 0:
            # Turn in place
            wheel1 = th * self.wheel_track  * self.gear_reduction
            wheel2 = wheel1
            wheel3 = wheel1
        elif th == 0:
            # Pure forward/backward motion
            wheel1 = -0.5 * x - 0.86603 * y
            wheel2 = x
            wheel3 = -0.5 * x + 0.86603 * y
        else:
            # Rotation about a point in space
            wheel1 = -0.5 * x - 0.86603 * y + th * self.wheel_track  * self.gear_reduction
            wheel2 = x + th * self.wheel_track  * self.gear_reduction
            wheel3 = -0.5 * x + 0.86603 * y + th * self.wheel_track  * self.gear_reduction

        self.v_des_wheel1 = int(wheel1 * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_wheel2 = int(wheel2 * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_wheel3 = int(wheel3 * self.ticks_per_meter / self.arduino.PID_RATE)


#        x = req.linear.x         # m/s
#        th = req.angular.z       # rad/s
#
#        if x == 0:
            # Turn in place
#            right = th * self.wheel_track  * self.gear_reduction / 2.0
#            left = -right
#        elif th == 0:
            # Pure forward/backward motion
#            left = right = x
#        else:
            # Rotation about a point in space
#            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
#            right = x + th * self.wheel_track  * self.gear_reduction / 2.0

#        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
#        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)
