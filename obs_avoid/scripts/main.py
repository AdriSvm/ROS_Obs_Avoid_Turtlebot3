#!/usr/bin/env python3
import numpy as np
import rospy,time, tf2_ros, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from collections import deque
from tf2_geometry_msgs import PointStamped

history_length = 10
regions_history = {
    'right':  deque(maxlen=history_length),
    'fright': deque(maxlen=history_length),
    'front':  deque(maxlen=history_length),
    'fleft':  deque(maxlen=history_length),
    'left':   deque(maxlen=history_length),
    'rear':   deque(maxlen=history_length),
}
actions_history_length = 30
actions_history = deque(maxlen=actions_history_length)

class ObstacleAvoider:

    def __init__(self,goal_x=3.0,goal_y=-2.0,goal_z=0.0):
        time.sleep(0.2)
        rospy.init_node('obs_avoid', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.goal = Point(x=goal_x, y=goal_y, z=goal_z)  # Ejemplo de objetivo
        rospy.spin()

    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
            return trans.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def calculate_goal_direction(self, position):
        dx = self.goal.x - position.x
        dy = self.goal.y - position.y
        return math.atan2(dy, dx)


    @staticmethod
    def calculate_velocity(regions, speed=1, turn=0.0):
        # Analizar el historial para tomar decisiones
        average_distances = {region: np.mean(regions_history[region]) for region in regions_history}
        min_distances = {region: min(regions_history[region]) for region in regions_history}

        # Ejemplo de decisión basada en el promedio de distancias
        print({i:round(j,2) for i,j in regions.items()})
        turning = None
        if regions['front'] < 0.1:
            if regions['rear'] > 0.3:
                speed = -0.1
                turn = 0
                turning = 'none'
                state_desc = "Heavy obstacle, going backwards"
            else:
                speed = 0
                if average_distances['fleft'] > average_distances['fright']:
                    turn,turning = -1.0* max(average_distances['fleft'],0.5),'left'
                else:
                    turn, turning = 1.0 * max(average_distances['fright'], 0.5), 'right'
                state_desc = "Heavy obstacle, rotating"

        elif regions['front'] < 1.4:
            speed = speed * max(min_distances['front'],1)**2
            if average_distances['fleft'] < 0.4 and average_distances['fright'] > 0.9:
                turn = 0.4
                turning = 'right'
            elif average_distances['fright'] < 0.4 and average_distances['fleft'] > 0.9:
                turn = -0.4
            else:
                if average_distances['fleft'] > average_distances['fright']:
                    turn = -0.7
                    turning = 'left'
                else:
                    turn = 0.7
                    turning = 'right'
            state_desc = "Light obstacle"
        else:
            state_desc = "No obstacle"
            if min_distances['fleft'] < 1.2 or min_distances['fright'] < 1.2:
                speed = speed * min(min_distances['front'],3)**2
                if regions['fleft'] < regions['fright'] and average_distances['fleft'] < 0.1:
                        turn = 0.5 * max(average_distances['fleft'],0.75)
                        turning = 'right'
                elif regions['fleft'] > regions['fright'] and average_distances['fright'] < 0.1:
                        turn = -0.5 * max(average_distances['fright'],0.75)
                        turning = 'left'

        lefts = sum([1 if action == 'left' else 0 for action in actions_history])
        rights = sum([1 if action == 'right' else 0 for action in actions_history])
        last_turn = actions_history[-1] if actions_history else None

        if last_turn == 'left' and lefts < len(actions_history)*0.7 and rights > len(actions_history)*0.2:
            turn = -abs(turn)
            turning = 'left'
            state_desc = "Avoiding right turn"

        if last_turn == 'right' and rights < len(actions_history)*0.7 and lefts > len(actions_history)*0.2:
            turn = abs(turn)
            turning = 'right'
            state_desc = "Avoiding left turn"

        actions_history.append(turning)

        return speed, turn, state_desc

    def take_action(self, regions, vel_normal_linear=0.4, mode='assertive'):
        msg = Twist()
        linear_x, angular_z, state_description = ObstacleAvoider.calculate_velocity(regions, vel_normal_linear)
        rospy.loginfo(f"Setting speed: linear={linear_x}, angular={angular_z}, state={state_description}")
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.pub.publish(msg)

    def callback(self, data):
        d = [0 if x < 0.05 else x for x in data.ranges[-90::]+data.ranges[:90]]
        if not any(d):
            print(1)
            regions = {
                'right': min(data.ranges[55:90] or [float('inf')]),
                'fright': min(data.ranges[25:55] or [float('inf')]),
                'front': min(data.ranges[-25::] + data.ranges[0:25] or [float('inf')]),
                'fleft': min(data.ranges[-55:-25] or [float('inf')]),
                'left': min(data.ranges[-90:-55] or [float('inf')]),
                'rear': min(data.ranges[150:210] or [float('inf')]),
            }
        else:
            print(2)
            data.ranges = [float('inf') if x == 0 else x for x in data.ranges]
            regions = {
                'right': min(data.ranges[55:90] or [float('inf')]),
                'fright': min(data.ranges[25:55] or [float('inf')]),
                'front': min(data.ranges[-25::] + data.ranges[0:25] or [float('inf')]),
                'fleft': min(data.ranges[-55:-25] or [float('inf')]),
                'left': min(data.ranges[-90:-55] or [float('inf')]),
                'rear': min(data.ranges[150:210] or [float('inf')]),
            }


        for region in regions_history:
            regions_history[region].append(regions[region])

        self.take_action(regions)


if __name__ == '__main__':
    ObstacleAvoider()
