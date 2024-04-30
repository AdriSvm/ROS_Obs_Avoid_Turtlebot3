#!/usr/bin/env python3

class ObstacleAvoider:
    regions = {
        "front": (-25,25),
        "rear": (155,205),
        "front_right": (26, 66),
        "rear_right": (116, 156),
        "right": (67, 115),
        "front_left": (-65, -25),
        "rear_left": (-115, -65),
        "left": (-155, -115)
    }

    def __init__(self,vel_publisher,scan_subscriber,obstacle_threshold=0.5,normal_lin_vel=1):
        self.vel_publisher = vel_publisher
        self.scan_subscriber = scan_subscriber
        self.obstacle_threshold = obstacle_threshold
        self.normal_lin_vel = normal_lin_vel


