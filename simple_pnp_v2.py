#!/usr/bin/env python

import sys
import copy
import rospy
import rospkg
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
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
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

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('blank_move_group',
                    anonymous=True, disable_signals=False)
    moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "right_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

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
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
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

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
   
    group.clear_pose_targets()

    
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

   

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

   
    group.execute(plan, wait=True)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    
  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "left_inner_finger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

sdf_cube = """<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>0</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="stairs_collision0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <surface>
          <bounce />
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000000.0</kp>
              <kd>1.0</kd>
              <min_depth>0.0</min_depth>
              <max_vel>0.0</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="stairs_visual0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""


def create_cube_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the cube given.
    modelname: name of the model for gazebo
    px py pz: position of the cube (and it's collision cube)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cube"""
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
    tester = MoveGroupPythonIntefaceTutorial()
    pose_list = []

    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
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

    #gripper_to_pos(0)

    sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.74, 0.02, 0.15)

    sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.74, 0.02, -0.11)

    gripper_to_pos(50)

    sleep(5.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.72, 0.0, 0.15)

    sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.8, 0.3, -0.05)

    gripper_to_pos(0)

  except rospy.ROSInterruptException:
    delete_gazebo_models()
    return
  except KeyboardInterrupt:
    delete_gazebo_models()
    return

if __name__ == '__main__':
  main()
