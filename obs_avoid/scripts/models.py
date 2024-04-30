#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider:


    def __init__(self):
        rospy.init_node('obs_avoid', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        rospy.spin()

    @staticmethod
    def calculate_velocity(regions,speed=0.5,turn=0.0):
        if regions['front'] < 0.6:
            # Frente bloqueado, se requiere una acción inmediata
            speed = 0
            if regions['fleft'] > regions['fright']:
                turn = 1.0  # Giro fuerte a la izquierda
            else:
                turn = -1.0  # Giro fuerte a la derecha
            state_desc = "Heavy obstacle"
        else:
            # Frente despejado o parcialmente despejado
            if regions['fleft'] < 1.2 or regions['fright'] < 1.2:
                # Obstáculo cercano a un lado
                if regions['fleft'] < regions['fright']:
                    turn = -0.5
                else:
                    turn = 0.5
                state_desc = "Light obstacle"
                speed = 0.3
            else:
                # Velocidad normal si todo está despejado
                speed = 0.5
                turn = 0.0
                state_desc = "No obstacle"

        return speed, turn,state_desc

    def take_action(self,regions,vel_normal_linear=1,mode='assertive'):
        msg = Twist()
        linear_x,angular_z,state_description = ObstacleAvoider.calculate_velocity(regions,vel_normal_linear)
        rospy.loginfo(f"Setting speed: linear={linear_x}, angular={angular_z}, state={state_description}")
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def callback(self,data):
        regions = {
            'right': min(min(data.ranges[0:143]), 3),
            'fright': min(min(data.ranges[144:287]), 3),
            'front': min(min(data.ranges[288:431]), 3),
            'fleft': min(min(data.ranges[432:575]), 3),
            'left': min(min(data.ranges[576:719]), 3),
        }
        self.take_action(regions)




