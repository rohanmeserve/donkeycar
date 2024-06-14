'''
Classes that allow more frequent position updates than the GPS alone

PositionEstimator calculates velocity and heading based on GPS readings, and if it has an IMU, then alters them with raw IMU readings (accel and gyro)

Written for
UCSD DSC190 WI/SP 2024
'''

import numpy as np
import math
import time

class PositionEstimator:
    '''
    A PositionEstimator which calculates a velocity and heading based on gps readings,
    and uses time elapsed to predict a new position.

    If an IMU is used, can adjust velocity using its accelerometer, and heading using its gyroscope.
    
    To improve frequency of position updates, needs the drive loop frequency to be increased.
    '''

    def __init__(self):
        # position, velocity, and orientation values
        self.x = 0.
        self.y = 0.
        self.vx = 0.
        self.vy = 0.
        self.yaw = 0.

        # last gps values received
        self.last_pos_x = 0.
        self.last_pos_y = 0.

        # timestamps for calculating time difference
        self.last_time = time.time()
        self.last_gps_time = time.time()

        self.start_time = time.time()


    def run(self, acl_x: float, acl_y: float, gyr_x: float, pos_x: float, pos_y: float):
        # updates stored velocity, orientation, and position using accelerometer and gyroscope readings

        curr_time = time.time()
        dt = curr_time - self.last_time

        # if IMU inputs are None (no IMU), then set to 0
        # this will essentially just project the position
        # based on last known positions and time elapsed
        if gyr_x is None:
            gyr_x = 0
        if acl_x is None:
            acl_x = 0
        if acl_y is None:
            acl_y = 0

        # update orientation
        self.yaw += float(gyr_x) * dt

        # lock yaw to 0->360 frame
        if self.yaw < 0:
            # handle negative case; -1deg = 359deg
            self.yaw += 2*math.pi
        self.yaw %= 2*math.pi


        # rotate accerlation vectors; apply trig then adjust signage according to heading
        ax = float(acl_x) * math.cos(self.yaw) - float(acl_y) * math.sin(self.yaw)
        ay = float(acl_x) * math.sin(self.yaw) - float(acl_y) * math.cos(self.yaw)

        # update velocity
        self.vx += ax * dt
        self.vy += ay * dt

        # update stored position
        self.x += self.vx * dt
        self.y += self.vy * dt

        # calculate total velocity
        velocity_total = math.sqrt(self.vx**2 + self.vy**2)

        # update time for next run
        self.last_time = curr_time

        # checks if the pos/x and pos/y from gps have recently been updated
        # if true, reset stored position and velocity to match actual values
        if pos_x != self.last_pos_x or pos_y != self.last_pos_y:
            # calculate time difference since last gps update
            gps_dt = curr_time - self.last_gps_time
            # reset position based on gps values
            pos_x = float(pos_x)
            pos_y = float(pos_y)
            self.x = pos_x
            self.y = pos_y
            # reset velocity based on gps delta
            self.vx = (pos_x - self.last_pos_x) / gps_dt
            self.vy = (pos_y - self.last_pos_y) / gps_dt

            # reset yaw
            if (pos_x - self.last_pos_x) > 0 and (pos_y - self.last_pos_y) > 0:
                # NE, 0 -> 90
                self.yaw = abs(math.atan((pos_y - self.last_pos_y) / (pos_x - self.last_pos_x)))
            elif (pos_x - self.last_pos_x) < 0  and (pos_y - self.last_pos_y) > 0:
                # NW, 90 - 180
                self.yaw = math.pi - abs(math.atan((pos_y - self.last_pos_y) / (pos_x - self.last_pos_x)))
            elif (pos_x - self.last_pos_x) < 0 and (pos_y - self.last_pos_y) < 0:
                # SW, 180 -> 270
                self.yaw = abs(math.atan((pos_y - self.last_pos_y) / (pos_x - self.last_pos_x))) + math.pi
            else:
                # SE, 270 -> 360
                self.yaw = (2*math.pi) - abs(math.atan((pos_y - self.last_pos_y) / (pos_x - self.last_pos_x)))
            # lock to 0 -> 360 frame
            self.yaw %= 2*math.pi
            self.last_gps_time = curr_time

        # updates last known gps position to current gps position
        self.last_pos_x = pos_x
        self.last_pos_y = pos_y


        # returns estimated x, estimated y, estimated orientation, absolute x accerlation, absolute y acceration, and total velocity
        return self.x, self.y, self.yaw, ax, ay, velocity_total
