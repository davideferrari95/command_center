#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from mir_actions.msg import MirMoveBaseAction, MirMoveBaseActionGoal, MirMoveBaseActionResult

class ROSNode:

    def __init__(self):

        rospy.init_node('experiment', anonymous=True)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
        self.joint_traj_pub = rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command', JointTrajectoryPoint, queue_size=1)
        self.move_base_pub = rospy.Publisher('/move_base/goal', MirMoveBaseActionGoal, queue_size=1)
        rospy.sleep(1)

        # ROS Rate
        self.rate = rospy.Rate(10)

    def move_cmd_vel(self, vel:float=0.4, duration:float=10):

        # Move the Robot Forward
        twist = TwistStamped()
        twist.twist.linear.x = vel

        # Set the End Time
        end_time = rospy.Time.now() + rospy.Duration(duration)

        while rospy.Time.now() < end_time:

            # Continuously Publish the Twist Message
            twist.header.stamp = rospy.Time.now()
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        # Stop the Robot
        twist.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def move_home(self):

        # Move the Robot to Different Joint Positions

        pre_home = [0.31900060176849365, -1.2135346571551722, -2.366403404866354, -1.145710293446676, 1.577465534210205, -1.1766775290118616]
        home = [0.3845580518245697, -1.1221545378314417, -2.258941952382223, -1.3255141417132776, 1.5840853452682495, 0.34170883893966675]

        points = [pre_home, home]

        for point in points:

            # Create Joint Trajectory Point
            traj_point = JointTrajectoryPoint()
            traj_point.positions = point
            traj_point.time_from_start = rospy.Duration(2)

            # Publish Joint Trajectory Point
            self.joint_traj_pub.publish(traj_point)

            # Wait for 2.5 Seconds
            rospy.sleep(2.5)


    def pick(self):

        # Move the Robot to Different Joint Positions

        insert = [0.3178628385066986, -1.2579520384417933, -1.8901923338519495, -1.559265438710348, 1.5845295190811157, 0.2750196158885956]
        intermediate_insert = [0.20847608149051666, -1.7173107306109827, -1.470961872731344, -1.5207656065570276, 1.5850569009780884, 0.16566318273544312]
        pre_pick = [0.2072540670633316, -1.9722335974322718, -1.1424043814288538, -1.5942657629596155, 1.5850448608398438, 0.1643696427345276]
        pick = [0.2072540670633316, -1.9759605566607874, -1.0764101187335413, -1.6565359274493616, 1.5850569009780884, 0.1643936038017273]
        post_pick = [0.20729002356529236, -2.009348217641012, -0.8609531561480921, -1.8385208288775843, 1.585020899772644, 0.16435766220092773]
        intermediate_up = [0.3998068571090698, -1.2259700934039515, -1.6844695250140589, -1.7957871595965784, 1.5841816663742065, 0.3568730056285858]
        place_up = [0.31900060176849365, -1.2474711577044886, -2.407222572957174, -1.0707810560809534, 1.5773457288742065, -1.176689926777975]
        place_down = [0.31902456283569336, -1.4173806349383753, -2.526759926472799, -0.7814701239215296, 1.5774176120758057, -1.1766775290118616]

        points = [insert, intermediate_insert, pre_pick, pick, post_pick, intermediate_up, place_up, place_down]

        for point in points:

            # Create Joint Trajectory Point
            traj_point = JointTrajectoryPoint()
            traj_point.positions = point
            traj_point.time_from_start = rospy.Duration(2)

            # Publish Joint Trajectory Point
            self.joint_traj_pub.publish(traj_point)

            # Wait for 2.5 Seconds
            rospy.sleep(2.5)

    def move_to_me(self):

        # Move Back
        self.move_cmd_vel(-0.25,8.0)

        # Rotate z
        twist = TwistStamped()
        twist.twist.angular.z = 0.3

        # Set the End Time
        end_time = rospy.Time.now() + rospy.Duration(5)

        while rospy.Time.now() < end_time:

            # Continuously Publish the Twist Message
            twist.header.stamp = rospy.Time.now()
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        self.move_cmd_vel(0.3,4.0)

    def move_mir_to_point(self, x:float, y:float, z:float, qx:float, qy:float, qz:float, qw:float):

        msg = MirMoveBaseActionGoal()

        msg.goal.move_task = 1
        msg.goal.target_pose.header.frame_id = "map"
        msg.goal.target_pose.pose.position.x = x
        msg.goal.target_pose.pose.position.y = y
        msg.goal.target_pose.pose.position.z = z
        msg.goal.target_pose.pose.orientation.x = qx
        msg.goal.target_pose.pose.orientation.y = qy
        msg.goal.target_pose.pose.orientation.z = qz
        msg.goal.target_pose.pose.orientation.w = qw

        # msg.header = Header()
        # msg.goal_id = GoalID()
        # msg.goal.target_pose = PoseStamped()

        # msg.goal_id.id = "/MissionController-24-1722958451.320042224"

        # msg.goal.move_task = 1
        # msg.goal.target_pose.header.frame_id = "map"
        # msg.goal.target_pose.pose.position.x = 5.696000099182129
        # msg.goal.target_pose.pose.position.y = 5.958000183105469
        # msg.goal.target_pose.pose.position.z = 0.0
        # msg.goal.target_pose.pose.orientation.x = 0.0
        # msg.goal.target_pose.pose.orientation.y = 0.0
        # msg.goal.target_pose.pose.orientation.z = -0.708130392496646
        # msg.goal.target_pose.pose.orientation.w = 0.706081685941893
        
        # msg.goal.target_guid = "5b643e40-6427-11ee-b5d1-b8aeed7269f0"
        # msg.goal.goal_dist_threshold = 0.25
        # msg.goal.goal_orientation_threshold = 0.0
        
        # msg.goal.max_plan_time = 0.0
        # msg.goal.clear_costmaps = True
        # msg.goal.pause_command = False
        # msg.goal.continue_command = False
        # msg.goal.yaw = 0.0
        # msg.goal.collision_detection = False
        # msg.goal.collision_avoidance = False
        # msg.goal.disable_collision_check_dist = 0.0
        # msg.goal.max_linear_speed = 0.0
        # msg.goal.max_rotational_speed = 0.0
        # msg.goal.pid_dist_offset = 0.0
        # msg.goal.target_offset = 0.0
        # msg.goal.only_collision_detection = False
        # msg.goal.timeout = 0.0
        # msg.goal.pattern_type = 0
        # msg.goal.pattern_value = 0
        # msg.goal.only_track = False
        # msg.goal.same_goal = False
        # msg.goal.pose_frame = ''

        self.move_base_pub.publish(msg)

if __name__ == '__main__':

    # Initialize ROS Node
    node = ROSNode()

    rospy.loginfo("Moving MIR to Home")
    node.move_mir_to_point(5.75, 5.0, 0.0, 0.0, 0.0, 0.03025939011186214, 0.9995420798095787)
    rospy.wait_for_message('/move_base/result', MirMoveBaseActionResult)

    rospy.loginfo("Moving UR to Home Position")
    node.move_home()

    rospy.loginfo("Moving to Table")
    node.move_mir_to_point(10.899999618530273, 8.449999809265137, 0.0, 0.0, 0.0, 0.0, 1.0)
    rospy.wait_for_message('/move_base/result', MirMoveBaseActionResult)

    rospy.loginfo("Executing Pick")
    node.pick()

    rospy.loginfo("Moving to Me")
    node.move_mir_to_point(5.1620001792907715, 5.389999866485596, 0.0, 0.0, 0.0, -0.9991496669037134, 0.04123036655425822)
    rospy.wait_for_message('/move_base/result', MirMoveBaseActionResult)

    rospy.loginfo("Finished")
