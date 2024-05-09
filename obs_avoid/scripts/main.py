#!/usr/bin/env python3
import numpy as np
import rospy,time, tf2_ros, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from collections import deque
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
actions_history_length = 15
actions_history = deque(maxlen=actions_history_length)

goal_position = None

free_path_history_len = 5
free_path_history = deque(maxlen=free_path_history_len)

class ObstacleAvoider:

    def __init__(self,goal_x=-1.0,goal_y=-2.0,goal_z=0.0):
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
        return destination

    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
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
        rot = math.atan2(dy, dx) if -2 * math.pi < math.atan2(dy, dx) < 2 * math.pi else math.atan2(dy, dx) / (2 * math.pi)
        normalized_rot = rot / math.pi
        return normalized_rot

    def dist_to_goal(self,position):
        return math.sqrt((goal_position.pose.position.x - position.translation.x)**2 + (goal_position.pose.position.y - position.translation.y)**2)

    def goal_accomplished(self,position):
        return self.dist_to_goal(position) < 0.5

    @staticmethod
    def calculate_velocity(regions, speed=1, turn=0.0):
        # Analyse historic data to better decisions
        average_distances = {region: np.mean(regions_history[region]) for region in regions_history}
        min_distances = {region: min(regions_history[region]) for region in regions_history}

        # We'll use the average of historic data and also the actual data to make better decisions
        # There are mainly 3 modes, No obstacle, Light Obstacle and Heavy Obstacle
        # Only with No obstacle mode, the robot will try to reach the goal
        # print({i:round(j,2) for i,j in regions.items()})
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

        '''
        #TODO: Upgrade the decision making process on erratic turning movements
        if last_turn == 'left' and lefts < len(actions_history)*0.7 and rights > len(actions_history)*0.2:
            turn = -abs(turn)
            turning = 'left'
            state_desc = "Avoiding right turn"

        if last_turn == 'right' and rights < len(actions_history)*0.7 and lefts > len(actions_history)*0.2:
            turn = abs(turn)
            turning = 'right'
            state_desc = "Avoiding left turn"
            '''

        actions_history.append(turning)

        return speed, turn, state_desc

    def free_path(self,ranges,position):
        # Function to calculate if there exists a direct path to the goal, to optimise trajectory
        goal_dir = self.calculate_goal_direction(position)
        current_euler = tf.transformations.euler_from_quaternion(
            [position.rotation.x, position.rotation.y, position.rotation.z, position.rotation.w]
        )
        rotation = current_euler[2]

        dife = goal_dir - rotation
        dife_err = (dife+math.pi) % (2*math.pi) - math.pi
        index = int(dife_err * (len(ranges)/(2*math.pi)) )

        if index-10 < 0:
            data = ranges[index-10::] + ranges[0:index+10]
        else:
            data = ranges[index-10:index+10]

        global free_path_history
        if not any(data):
            free_path_history.append(False)
            return False if free_path_history.count(False) > len(free_path_history)*0.2 else True

        data = [x for x in data if x > 0.05 and not x == float('inf')]
        if not data or self.dist_to_goal(position) < (sum(data)/len(data)):
            free_path_history.append(True)
            return False if free_path_history.count(False) > len(free_path_history) * 0.2 else True


    def calculate_turn(self,data,regions, goal_direction,position,linear_x,turn_angle,state_description):

        if self.free_path(data.ranges,position):
            if abs(round(goal_direction,2)-round(position.rotation.z,2)) < 0.05:
                # Linear velocity is not changed, maintains the decision made by calculate_velocity
                if goal_direction > position.rotation.z:
                    turn_angle = 0.2 * min(abs(goal_direction - position.rotation.z), 1)
                else:
                    turn_angle = -0.2 * min(abs(goal_direction - position.rotation.z), 1)
            else:
                time.sleep(0.2)
                # Progressive linear velocity to not make heavy changes in velocity
                linear_x = 0 if abs(goal_direction - position.rotation.z) > 0.1 else linear_x * abs(goal_direction - position.rotation.z) / 2
                if goal_direction > position.rotation.z:
                    turn_angle = 0.2 * min(abs(goal_direction - position.rotation.z)*5, 1)
                else:
                    turn_angle = -0.2 * min(abs(goal_direction - position.rotation.z)*5, 1)
            state_description = "Free Path, Heading goal"
            return linear_x, turn_angle, state_description

        if state_description == 'No obstacle':
            # If there is no obstacle and a direct path couldn't be made, then the robot will try to rotate to the goal while moving forward
            if abs(round(goal_direction,2)-round(position.rotation.z,2)) < 0.05:
                turn_angle = 0
            else:
                linear_x=0
                if goal_direction > position.rotation.z:
                    turn_angle = 0.1 * min(abs(goal_direction - position.rotation.z)+1,1)
                else:
                    turn_angle = -0.1 * min(abs(goal_direction - position.rotation.z)+1,1)
            state_description = "Heading towards the goal"

        return linear_x,turn_angle, state_description

    def take_action(self, regions, data, vel_normal_linear=0.4, mode='assertive'):
        # Main action to take, will call to all the other functions to make a decision
        msg = Twist()
        pos = self.get_robot_position()

        if not pos:
            raise Exception("Robot position not found, Make sure the slam_gmapping node is running with map and link_footprint frames")

        global goal_position
        if not goal_position:
            goal_position = self.calculate_goal_position(pos,self.goal_x,self.goal_y,self.goal_z)

        goal_direction = self.calculate_goal_direction(pos)

        linear_x, turn_angle, state_description = ObstacleAvoider.calculate_velocity(regions, vel_normal_linear)
        linear_x, angular_z, state_description = self.calculate_turn(data,regions, goal_direction,pos,linear_x,turn_angle,state_description)

        if self.goal_accomplished(pos):
            rospy.loginfo("Goal accomplished")
            msg.linear.x = 0
            msg.angular.z = 0
            self.pub.publish(msg)
            return None

        # Thanks to this loginfo, we have a very rich information while the robot is moving, through the terminal
        rospy.loginfo(f"Setting speed: linear={str(linear_x)}, angular={str(angular_z)}, "
                      f"state={str(state_description)},\npose={str(round(pos.translation.x,3))},{str(round(pos.translation.y,3))},{str(round(pos.rotation.z,3))}, goal={str(round(goal_position.pose.position.x,3))},{str(round(goal_position.pose.position.y,3))},"
                      f"\ngoal_direction:{str(round(goal_direction,2))}, "
                      f"dist_to_goal:{str(round(self.dist_to_goal(pos),2))}")

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

        self.take_action(regions,data)


if __name__ == '__main__':

    x = rospy.get_param('goal_x',0)
    y = rospy.get_param('goal_y',0)
    rospy.loginfo(f"Initiating with goal: {x,y}")

    ObstacleAvoider(x,y)
