#!/usr/bin/env python

import sys
import copy
import rospy
import rospkg
import os
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from time import sleep
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gripper_to_position import gripper_to_pos
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    SpawnModelRequest,
    SpawnModelResponse
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from copy import deepcopy
from tf.transformations import quaternion_from_euler

## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal: A list of floats, a Pose or a PoseStamped
  @param: actual: list of floats, a Pose or a PoseStamped
  @param: tolerance: A float
  """
  
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

joint_state_topic = ['joint_states:=/robot/joint_states']

class PickAndPlace(object):
  def __init__(self):
    super(PickAndPlace, self).__init__()

    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('simple_pnp_gazebo',
                    anonymous=True, disable_signals=False)
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()

    eef_link = group.get_end_effector_link()

    group_names = robot.get_group_names()

    print robot.get_current_state()

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_pose_goal(self, ox, oy, oz, ow, px, py, pz):
    """
    Movement method to go to desired end effector pose
    @param: ox: Pose orientation for the x-axis (part of Quaternion)
    @param: oy: Pose orientation for the y-axis (part of Quaternion)
    @param: oz: Pose orientation for the z-axis (part of Quaternion)
    @param: ow: Pose orientation for the w (part of Quaternion)
    @param: px: Coordinate on the x-axis 
    @param: py: Coordinate on the y-axis
    @param: pz: Coordinate on the z-axis
    """

    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.orientation.w = ow
    pose_goal.position.x = px
    pose_goal.position.y = py
    pose_goal.position.z = pz
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
   
    group.clear_pose_targets()

    
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def display_trajectory(self, plan):
    """
    Display a movement plan / trajectory
    @param: plan: Plan to be displayed
    """

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
  
    display_trajectory_publisher.publish(display_trajectory);

   

  def execute_plan(self, plan):
    """
    Execute a movement plan
    @param: plan: Plan to be executed
    """
    group = self.group

   
    group.execute(plan, wait=True)

def create_cube_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """
    Create a cube to be spawned in gazebo
    @param: px: Position coordinate on x-axis
    @param: py: Position coordinate on y-axis
    @param: pz: Position coordinate on z-axis
    @param: rr: Roll rotation
    @param: rp: Pitch rotation
    @param: ry: Yaw rotation
    @param: sx: Cube size on x-axis
    @param: sy: Cube size on y-axis
    @param: sz: Cube size on z-axis
    """

    USERNAME = 'micahreich'    # Name of your user on your PC ... I'm not sure of an alternative to absolute paths

    with open('/home/' + USERNAME + '/catkin_ws/src/robotiq/robotiq_2f_gripper_control/models/cube.sdf', 'r') as file:
      sdf_cube = file.read().replace('\n', '')

    cube = deepcopy(sdf_cube)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = cube
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req

def main():
  try:
    tester = PickAndPlace()
    pose_list = []

    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)    # SPAWNING CUBE OBJECT IN GAZEBO
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    # Spawn object 1
    rospy.loginfo("Spawning cube1")
    req1 = create_cube_request("cube1",
                              0.7, 0.0, 0.77,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.0762, 0.0762, 0.0762)  # size
    spawn_srv.call(req1)

    gripper_to_pos(0, 60, 200, False)    # ACTIVATION STEP
    gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

    sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,    # GO TO WAYPOINT 1 (HOVER POS)
                             0.67, -0.01, 0.15)

    sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,    # GO TO WAYPOINT 2 (PLUNGE AND PICK)
                             0.67, -0.01, -0.12)

    gripper_to_pos(50, 60, 200, False)    # GRIPPER TO POSITION 50

    os.system('cd ~/catkin_ws/ | rosrun gazebo_ros_link_attacher attach.py')    # ATTACH CUBE AND SAWYER EEF

    sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,    # GO TO WAYPOINT 3 (TRAVEL TO PLACE DESTINATION)
                             0.67, 0.04, 0.15)

    sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,    # GO TO WAYPOINT 4 (PLACE)
                             0.8, 0.3, -0.05)

    os.system('cd ~/catkin_ws/ | rosrun gazebo_ros_link_attacher detach.py')    # DETACH CUBE AND SAWYER EEF

    gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,    # GO TO WAYPOINT 5 (RETURN TO HOVER POS)
                             0.67, -0.01, 0.15)

  except rospy.ROSInterruptException:
    delete_gazebo_models()
    return
  except KeyboardInterrupt:
    delete_gazebo_models()
    return

if __name__ == '__main__':
  main()
