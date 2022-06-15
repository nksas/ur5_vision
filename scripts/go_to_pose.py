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


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
        self._planning_group = "arm"
	self._gripper_group ="gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
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
#*******************************OBJECT-1**************************
    def object1():
	    roll = -1.606114
	    pitch = 0.067235
	    yaw = 2.288030
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_1 = geometry_msgs.msg.Pose()
	    ur5_pose_1.position.x = 0.548861
	    ur5_pose_1.position.y = -0.004245
	    ur5_pose_1.position.z = 0.873316
	    ur5_pose_1.orientation.x = quat[0]
	    ur5_pose_1.orientation.y = quat[1]
	    ur5_pose_1.orientation.z = quat[2]
	    ur5_pose_1.orientation.w = quat[3]
	    ur5.go_to_pose(ur5_pose_1)


	    roll = -1.604832
	    pitch = -0.023021
	    yaw = 2.220813
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_2 = geometry_msgs.msg.Pose()
	    ur5_pose_2.position.x = 0.566628
	    ur5_pose_2.position.y = -0.024518
	    ur5_pose_2.position.z = 0.811570
	    ur5_pose_2.orientation.x = quat[0]
	    ur5_pose_2.orientation.y = quat[1]
	    ur5_pose_2.orientation.z = quat[2]
	    ur5_pose_2.orientation.w = quat[3]
            ur5.go_to_pose(ur5_pose_2)
	    ur5.gripper("close")
	    roll = -1.576027
	    pitch = -0.022154
	    yaw = -1.641427
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_3 = geometry_msgs.msg.Pose()
	    ur5_pose_3.position.x = -0.144217
	    ur5_pose_3.position.y = 0.704728
	    ur5_pose_3.position.z = 0.909527
	    ur5_pose_3.orientation.x = quat[0]
	    ur5_pose_3.orientation.y = quat[1]
	    ur5_pose_3.orientation.z = quat[2]
	    ur5_pose_3.orientation.w = quat[3]
	    ur5.go_to_pose(ur5_pose_3)
	    ur5.gripper("open")
    def object2():
	    roll = -1.649460
	    pitch = 0.026926
	    yaw = -2.280846
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_1 = geometry_msgs.msg.Pose()
	    ur5_pose_1.position.x = 0.469466
	    ur5_pose_1.position.y = 0.209983
	    ur5_pose_1.position.z = 0.864190
	    ur5_pose_1.orientation.x = quat[0]
	    ur5_pose_1.orientation.y = quat[1]
	    ur5_pose_1.orientation.z = quat[2]
	    ur5_pose_1.orientation.w = quat[3]
	    ur5.go_to_pose(ur5_pose_1)

	    roll = -1.649418
	    pitch = 0.027087
	    yaw = -2.280782
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_2 = geometry_msgs.msg.Pose()
	    ur5_pose_2.position.x = 0.467401
	    ur5_pose_2.position.y = 0.213616
	    ur5_pose_2.position.z = 0.813949
	    ur5_pose_2.orientation.x = quat[0]
	    ur5_pose_2.orientation.y = quat[1]
	    ur5_pose_2.orientation.z = quat[2]
	    ur5_pose_2.orientation.w = quat[3]
            ur5.go_to_pose(ur5_pose_2)
	    ur5.gripper("object22")
	    roll = -1.575456
	    pitch = 0.032681
	    yaw = -1.641537
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_3 = geometry_msgs.msg.Pose()
	    ur5_pose_3.position.x = 0.073693
	    ur5_pose_3.position.y = -0.711680
	    ur5_pose_3.position.z = 0.928996
	    ur5_pose_3.orientation.x = quat[0]
	    ur5_pose_3.orientation.y = quat[1]
	    ur5_pose_3.orientation.z = quat[2]
	    ur5_pose_3.orientation.w = quat[3]
	    ur5.go_to_pose(ur5_pose_3)
	    ur5.gripper("open")
    def object3():
	    roll = -1.548783
	    pitch = -0.003629
	    yaw = 2.974844
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_1 = geometry_msgs.msg.Pose()
	    ur5_pose_1.position.x = 0.563835
	    ur5_pose_1.position.y = -0.257399
	    ur5_pose_1.position.z = 0.911942
	    ur5_pose_1.orientation.x = quat[0]
	    ur5_pose_1.orientation.y = quat[1]
	    ur5_pose_1.orientation.z = quat[2]
	    ur5_pose_1.orientation.w = quat[3]
	    ur5.go_to_pose(ur5_pose_1)

	    roll = -1.548712
	    pitch = -0.003647
	    yaw = 2.974644
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_2 = geometry_msgs.msg.Pose()
	    ur5_pose_2.position.x = 0.563460
	    ur5_pose_2.position.y = -0.258447
	    ur5_pose_2.position.z = 0.864347
	    ur5_pose_2.orientation.x = quat[0]
	    ur5_pose_2.orientation.y = quat[1]
	    ur5_pose_2.orientation.z = quat[2]
	    ur5_pose_2.orientation.w = quat[3]
            ur5.go_to_pose(ur5_pose_2)
	    ur5.gripper("object33")
	    roll = -1.575456
	    pitch = 0.032681
	    yaw = -1.641537
	    quat = quaternion_from_euler(roll, pitch, yaw)

	    ur5_pose_3 = geometry_msgs.msg.Pose()
	    ur5_pose_3.position.x = 0.073693
	    ur5_pose_3.position.y = -0.711680
	    ur5_pose_3.position.z = 0.928996
	    ur5_pose_3.orientation.x = quat[0]
	    ur5_pose_3.orientation.y = quat[1]
	    ur5_pose_3.orientation.z = quat[2]
	    ur5_pose_3.orientation.w = quat[3]
	    ur5.go_to_pose(ur5_pose_3)
	    ur5.gripper("open")
#*****************************************************************
   

    while not rospy.is_shutdown():
	ur5.go_to_predefined_pose("up")
	object1()
        rospy.sleep(3)
	object2()
        rospy.sleep(3)
	object3()
        rospy.sleep(3)

    


if __name__ == '__main__':
    main()
