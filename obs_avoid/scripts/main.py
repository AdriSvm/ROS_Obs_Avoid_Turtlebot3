#!/usr/bin/env python3
import numpy as np
import rospy,time, tf2_ros, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from collections import deque
from tf2_geometry_msgs import PointStamped
import tf.transformations

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

goal_position = None

class ObstacleAvoider:

    def __init__(self,goal_x=3.0,goal_y=3.0,goal_z=0.0):
        time.sleep(0.2)
        rospy.init_node('obs_avoid', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        robot_pos = self.get_robot_position()
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_z = goal_z

        #if not robot_pos:
        #    raise Exception("Robot position not found, Make sure the slam_gmapping node is running with map and link_footprint frames")

        global goal_position
        goal_position = self.calculate_goal_position(robot_pos,goal_x,goal_y,goal_z)

        rospy.spin()

    def calculate_goal_position(self,robot_position,x,y,rotz):
        if not robot_position:
            return None
        else:
            current_position = robot_position.translation
            current_orientation = robot_position.rotation

        final_x = current_position.x + x
        final_y = current_position.y + y

        current_euler = tf.transformations.euler_from_quaternion(
            [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        )
        new_rotation = tf.transformations.quaternion_from_euler(
            current_euler[0], current_euler[1], current_euler[2] + rotz
        )

        destination = PoseStamped()
        destination.header.stamp = rospy.Time.now()
        destination.header.frame_id = 'map'
        destination.pose.position.x = final_x
        destination.pose.position.y = final_y
        destination.pose.position.z = current_position.z  # Asumiendo que z no cambia
        destination.pose.orientation.x = new_rotation[0]
        destination.pose.orientation.y = new_rotation[1]
        destination.pose.orientation.z = new_rotation[2]
        destination.pose.orientation.w = new_rotation[3]
        print(destination.pose.position.x,destination.pose.position.y)
        return destination

    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            return trans.transform
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Error al buscar transformación: {e}")
        except tf2_ros.ConnectivityException as e:
            rospy.logerr(f"Problema de conectividad con tf2: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Error de extrapolación en tf2: {e}")
        return None

    def calculate_goal_direction(self, position):

        dx = goal_position.pose.position.x - position.translation.x
        dy = goal_position.pose.position.y - position.translation.y
        print("diffs",dx,dy)
        return math.atan2(dy, dx)

    def dist_to_goal(self,position):
        return math.sqrt((goal_position.pose.position.x - position.translation.x)**2 + (goal_position.pose.position.y - position.translation.y)**2)

    def goal_accomplished(self,position):
        return self.dist_to_goal(position) < 0.5

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

    @staticmethod
    def calculate_turn(regions, goal_direction,position,linear_x):
        # Comenzamos asumiendo que el robot puede avanzar hacia el objetivo directamente
        if abs(round(goal_direction,2)-round(position.rotation.z,2)) < 0.2:
            turn_angle = 0
        else:
            if goal_direction > position.rotation.z:
                turn_angle = 0.2 * min(abs(goal_direction - position.rotation.z)+1,1)
            else:
                turn_angle = -0.2 * min(abs(goal_direction - position.rotation.z)+1,1)
        state_description = "Heading towards the goal"

        # Verificar si hay obstáculos en la dirección del objetivo
        if regions['front'] < 1.0:  # Si hay un obstáculo cercano al frente
            if regions['fleft'] < regions['fright']:
                # Si el lado izquierdo está más libre que el derecho, gira a la izquierda
                turn_angle += 0.5  # Ajustar ángulo adecuadamente
                state_description = "Obstacle ahead, turning left"
            else:
                # Si el lado derecho está más libre que el izquierdo, gira a la derecha
                turn_angle -= 0.5  # Ajustar ángulo adecuadamente
                state_description = "Obstacle ahead, turning right"
        elif regions['fleft'] < 0.5:
            # Si hay un obstáculo cercano a la izquierda, ajusta el giro hacia la derecha
            turn_angle -= 0.3
            state_description = "Obstacle on left, turning right"
        elif regions['fright'] < 0.5:
            # Si hay un obstáculo cercano a la derecha, ajusta el giro hacia la izquierda
            turn_angle += 0.3
            state_description = "Obstacle on right, turning left"

        return linear_x,turn_angle, state_description

    def take_action(self, regions, vel_normal_linear=0.4, mode='assertive'):
        msg = Twist()
        pos = self.get_robot_position()

        if not pos:
            raise Exception("Robot position not found, Make sure the slam_gmapping node is running with map and link_footprint frames")

        global goal_position
        if not goal_position:
            goal_position = self.calculate_goal_position(pos,self.goal_x,self.goal_y,self.goal_z)

        goal_direction = self.calculate_goal_direction(pos)



        linear_x, _, state_description = ObstacleAvoider.calculate_velocity(regions, vel_normal_linear)
        linear_x, angular_z, state_description = self.calculate_turn(regions, goal_direction,pos,linear_x)

        if self.goal_accomplished(pos):
            rospy.loginfo("Goal accomplished")
            msg.linear.x = 0
            msg.angular.z = 0
            self.pub.publish(msg)
            return None
        rospy.loginfo(f"Setting speed: linear={str(linear_x)}, angular={str(angular_z)}, "
                      f"state={str(state_description)},\npose={str(round(pos.translation.x,3))},{str(round(pos.translation.y,3))},{str(round(pos.rotation.z,3))}, goal={str(round(goal_position.pose.position.x,3))},{str(round(goal_position.pose.position.y,3))},"
                      f"\ngoal_direction:{str(goal_direction)}, "
                      f"dist_to_goal:{str(self.dist_to_goal(pos))}")
        msg.linear.x = linear_x
        msg.angular.z = angular_z



        self.pub.publish(msg)

    def callback(self, data):
        d = [0 if x < 0.05 else x for x in data.ranges[-90::]+data.ranges[:90]]
        if not any(d):
            regions = {
                'right': min(data.ranges[55:90] or [float('inf')]),
                'fright': min(data.ranges[25:55] or [float('inf')]),
                'front': min(data.ranges[-25::] + data.ranges[0:25] or [float('inf')]),
                'fleft': min(data.ranges[-55:-25] or [float('inf')]),
                'left': min(data.ranges[-90:-55] or [float('inf')]),
                'rear': min(data.ranges[150:210] or [float('inf')]),
            }
        else:
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
