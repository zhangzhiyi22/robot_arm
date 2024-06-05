#!/usr/bin/env python3


# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

     # Initialize the marker publisher
      self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)


      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)



  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)
  
  def publish_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for point in points:
            p = Point()
            p.x, p.y, p.z = point[0], point[1], point[2]
            marker.points.append(p)

        self.marker_pub.publish(marker)

  
def main():
    
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success

  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass


  
  if success:

    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")

     # 定义正四面体的顶点，基于home位置(0.435, 0.194, 0.457)
    vertices = [
        [0.435, 0.194, 0.457],         # 顶点 1 (home)
        [0.435 + 0.1, 0.194, 0.457],   # 顶点 2 (沿X轴正方向)
        [0.435, 0.194 + 0.1, 0.457],   # 顶点 3 (沿Y轴正方向)
        [0.435, 0.194, 0.457 + 0.1]    # 顶点 4 (沿Z轴正方向)
    ]

    # 定义正四面体的边
    edges = [
        (0, 1), (1, 2), (2, 0),  # 底面三条边
        (0, 3), (1, 3), (2, 3)   # 三条从底面到顶点的边
    ]

    # 生成插值点
    num_points_per_edge = 10
    interpolated_poses = []

    for edge in edges:
        start, end = vertices[edge[0]], vertices[edge[1]]
        for t in np.linspace(0, 1, num_points_per_edge):
            interp_pose = [
                start[i] * (1 - t) + end[i] * t
                for i in range(3)
            ] 
            interpolated_poses.append(interp_pose)

    # 发布Marker显示轨迹
    example.publish_marker(interpolated_poses)

    # 发送插值点
    for i, pose in enumerate(interpolated_poses):
       

        actual_pose = example.get_cartesian_pose()
        actual_pose.position.x = pose[0]
        actual_pose.position.y = pose[1]
        actual_pose.position.z = pose[2]
        success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)


        if not success:
            break




    rospy.loginfo("Reaching Cartesian Pose...")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.z -= 0.2
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)

    

    rospy.loginfo("Reach Cartesian Pose with constraints...")
    # Get actual pose
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.y -= 0.3
    
    # Orientation constraint (we want the end effector to stay the same orientation)
    constraints = moveit_msgs.msg.Constraints()
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.orientation = actual_pose.orientation
    constraints.orientation_constraints.append(orientation_constraint)

    # Send the goal
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)



  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
