#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import quaternion_from_euler
import tf 
from std_srvs.srv import Empty
import roslaunch
from object_msgs.msg import ObjectPose


rospy.wait_for_service('/clear_octomap') 
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
pub = rospy.Publisher('/detection_info', ObjectPose, queue_size=3)

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('pickandplace', anonymous=True)
        self._planning_group = "arm"
	self._gripper_group ="gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
	self._group2 = moveit_commander.MoveGroupCommander(self._gripper_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
	self._planning_frame2 = self._group2.get_planning_frame()
        self._eef_link2 = self._group2.get_end_effector_link()
        self._group_names2 = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
   
    # Function to control predefine pose
    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def gripper(self,grip_name):
        self._group2 = moveit_commander.MoveGroupCommander("gripper")
        self._group2.set_named_target(grip_name)
        self._group2.go()
	
    # Function to control gripper angle 
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group2.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group2.set_joint_value_target(arg_list_joint_angles)
        self._group2.plan()
        flag_plan = self._group2.go(wait=True)

        list_joint_values = self._group2.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group2.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Function to go to desired pose
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')



def main():

    ur5 = Ur5Moveit()


#*************************Function to pick and place object1*******************************************************
    def object1():

    	# setting up roll, pitch, yaw for object1
 	    #roll = -2.808738
	    #pitch = 0.000392
	    #yaw = -2.996251
	    roll = -2.531743
	    pitch = 0.050023
	    yaw = -3.138081
	    quat = quaternion_from_euler(roll, pitch, yaw) # converting the r,p,y values to quaternion values.

	    # placing the arm above the object
	    ur5_pose_1 = geometry_msgs.msg.Pose()
	    ur5_pose_1.position.x = 0.274035
	    ur5_pose_1.position.y = 0.109887
	    ur5_pose_1.position.z =  1.131241
	    ur5_pose_1.orientation.x = quat[0]
	    ur5_pose_1.orientation.y = quat[1]
	    ur5_pose_1.orientation.z = quat[2]
	    ur5_pose_1.orientation.w = quat[3]
	    ur5.go_to_pose(ur5_pose_1)
    listener = tf.TransformListener ()
    while not rospy.is_shutdown():
	
	object1() # Pick and place of object1.
    	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    	roslaunch.configure_logging(uuid)
    	launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/neeraj/catkin_sb/src/sahayak_bot/obj_recog/launch/find_3d_object.launch"])
    	launch.start()

    	try:
            (translation, rotation) = listener.lookupTransform('base_link','object_40', rospy.Time(0))
            (translation1, rotation1) = listener.lookupTransform('base_link','object_41', rospy.Time(0))
            (translation2, rotation2) = listener.lookupTransform('base_link','object_42', rospy.Time(0))
            coor1 = []
            coor2 = []
            coor3 = []
            (trans,rot) = listener.lookupTransform('ebot_base', 'object_40', rospy.Time(0))
            coor1 = trans
            (trans,rot) = listener.lookupTransform('ebot_base', 'object_41', rospy.Time(0))
            coor2 = trans
            (trans,rot) = listener.lookupTransform('ebot_base', 'object_42', rospy.Time(0))
            coor3 = trans
            obj1 = ObjectPose()
            obj1.name = "Coke"
            obj1.pose.pose.position.x = translation1[0]
            obj1.pose.pose.position.y = translation1[1]
            obj1.pose.pose.position.z = translation1[2]
            obj1.pose.pose.orientation.x = 0	
            obj1.pose.pose.orientation.y = 0
            obj1.pose.pose.orientation.z = 0
            obj1.pose.pose.orientation.w = 0	

            obj2 = ObjectPose()
            obj2.name = "Battery"
            obj2.pose.pose.position.x = translation2[0]
            obj2.pose.pose.position.y = translation2[1]
            obj2.pose.pose.position.z = translation2[2]
            obj2.pose.pose.orientation.x = 0	
            obj2.pose.pose.orientation.y = 0
            obj2.pose.pose.orientation.z = 0
            obj2.pose.pose.orientation.w = 0	


            obj3 = ObjectPose()
            obj3.name = "Glue"
            obj3.pose.pose.position.x = translation[0]
            obj3.pose.pose.position.y = translation[1]
            obj3.pose.pose.position.z = translation[2]
            obj3.pose.pose.orientation.x = 0	
            obj3.pose.pose.orientation.y = 0
            obj3.pose.pose.orientation.z = 0
            obj3.pose.pose.orientation.w = 0
            count = 0
     	    
            if(count==0):
		
                pub.publish(obj1)
                pub.publish(obj2)
                pub.publish(obj3)
                count = count + 1	
            	    
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    	rospy.sleep(5)
    	launch.shutdown()
    	clear_octomap()
	
        pose_target = geometry_msgs.msg.Pose()
        drop_target = geometry_msgs.msg.Pose()
        safe_target = geometry_msgs.msg.Pose()
        droll = -1.638495
        dpitch = 0.049783
        dyaw = -3.137088
        roll = -2.8
        pitch = -0.020220
        yaw = -3.141322
        quat = quaternion_from_euler(roll, pitch, yaw)
        dquat = quaternion_from_euler(droll, dpitch, dyaw)
        drop_target.position.x = round(0.519657,6)
        drop_target.position.y = round(0.083696,6)
        drop_target.position.z = round(1.118348,6)
        drop_target.orientation.x = round(dquat[0],6)
        drop_target.orientation.y = round(dquat[1],6)
        drop_target.orientation.z = round(dquat[2],6)
        drop_target.orientation.w = round(dquat[3],6)
        safe_target.position.x = round(0.274035,6)
        safe_target.position.y = round(0.109887,6)
        safe_target.position.z = round(1.131241,6)
        safe_target.orientation.x = round(quat[0],6)
        safe_target.orientation.y = round(quat[1],6)
        safe_target.orientation.z = round(quat[2],6)
        safe_target.orientation.w = round(quat[3],6)



        pose_target.position.x = round(coor1[0],6)
        pose_target.position.y = round(coor1[1]-0.29,6)
        pose_target.position.z = round(coor1[2]+0.05,6)
        pose_target.orientation.x = round(quat[0],6)
        pose_target.orientation.y = round(quat[1],6)
        pose_target.orientation.z = round(quat[2],6)
        pose_target.orientation.w = round(quat[3],6)
        ur5.go_to_pose(pose_target) #initial
        pose_target.position.y = round(coor1[1]-0.19,6)
        ur5.go_to_pose(pose_target) #approach
        ur5.set_joint_angles([math.radians(15)]) #close
        rospy.sleep(2) #wait
        pose_target.position.z = round(coor1[2]+0.3,6) 
        ur5.go_to_pose(pose_target) #lift upward
        ur5.go_to_pose(drop_target) #drop
        ur5.set_joint_angles([math.radians(0)])
        ur5.go_to_pose(safe_target) #initial
        
        pose_target.position.x = round(coor2[0]+0.01,6)
        pose_target.position.y = round(coor2[1]-0.3,6)
        pose_target.position.z = round(coor2[2]+0.10,6)
        ur5.go_to_pose(pose_target)
        pose_target.position.y = round(coor2[1]-0.19,6)
        pose_target.position.z = round(coor2[2]+0.07,6)
        ur5.go_to_pose(pose_target)
        ur5.set_joint_angles([math.radians(17)])
        rospy.sleep(2)
        pose_target.position.z = round(coor2[2]+0.3,6)
        ur5.go_to_pose(pose_target)
        ur5.go_to_pose(drop_target)
        ur5.set_joint_angles([math.radians(0)])
        ur5.go_to_pose(safe_target)
        
        pose_target.position.x = round(coor3[0],6)
        pose_target.position.y = round(coor3[1]-0.29,6)
        pose_target.position.z = round(coor3[2]+0.07,6)
        ur5.go_to_pose(pose_target)
        pose_target.position.y = round(coor3[1]-0.19,6)
        ur5.go_to_pose(pose_target)
        ur5.set_joint_angles([math.radians(18)])
        rospy.sleep(3)
        pose_target.position.z = round(coor3[2]+0.3,6)
        ur5.go_to_pose(pose_target)
        ur5.go_to_pose(drop_target)
        ur5.set_joint_angles([math.radians(0)])
            

        sys.exit() # kill the node once task is done

if __name__ == '__main__':
    main()
