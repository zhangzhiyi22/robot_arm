#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
        
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander('arm')
        
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        self.arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # 初始化Marker
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        self.arm.set_max_velocity_scaling_factor(1.0)  # 将速度缩放因子设置为最大值1.0
        self.arm.set_max_acceleration_scaling_factor(1.0)  # 将加速度缩放因子设置为最大值1.0

        
        # 控制机械臂运动到之前设置的“home”姿态
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
        # 获取当前位姿数据最为机械臂运动的起始位姿
        self.start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # 定义正方体的顶点，基于home位置
        self.vertices = [
            [self.start_pose.position.x, self.start_pose.position.y, self.start_pose.position.z],  # 顶点 1 (home)
            [self.start_pose.position.x + 0.1, self.start_pose.position.y, self.start_pose.position.z],  # 顶点 2 (沿X轴正方向)
            [self.start_pose.position.x + 0.1, self.start_pose.position.y + 0.1, self.start_pose.position.z],  # 顶点 3 (沿X轴和Y轴正方向)
            [self.start_pose.position.x, self.start_pose.position.y + 0.1, self.start_pose.position.z],  # 顶点 4 (沿Y轴正方向)
            [self.start_pose.position.x, self.start_pose.position.y, self.start_pose.position.z + 0.1],  # 顶点 5 (沿Z轴正方向)
            [self.start_pose.position.x + 0.1, self.start_pose.position.y, self.start_pose.position.z + 0.1],  # 顶点 6 (沿X轴和Z轴正方向)
            [self.start_pose.position.x + 0.1, self.start_pose.position.y + 0.1, self.start_pose.position.z + 0.1],  # 顶点 7 (沿X轴、Y轴和Z轴正方向)
            [self.start_pose.position.x, self.start_pose.position.y + 0.1, self.start_pose.position.z + 0.1]  # 顶点 8 (沿Y轴和Z轴正方向)
        ]

        # 定义正方体的边（顺序包括重复点以形成闭合路径）
        self.edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # 底面四条边
            (4, 5), (5, 6), (6, 7), (7, 4),  # 顶面四条边
            (0, 4), (1, 5), (2, 6), (3, 7)   # 竖直方向四条边
        ]

        self.publish_marker(self.vertices)
        self.follow_edges()

        # 控制机械臂回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    
    def publish_marker(self, vertices):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for edge in self.edges:
            start = vertices[edge[0]]
            end = vertices[edge[1]]
            start_point = Point()
            start_point.x, start_point.y, start_point.z = start
            end_point = Point()
            end_point.x, end_point.y, end_point.z = end
            marker.points.append(start_point)
            marker.points.append(end_point)

        self.marker_pub.publish(marker)

    def follow_edges(self):
        waypoints = []

        for edge in self.edges:
            start, end = self.vertices[edge[0]], self.vertices[edge[1]]
            pose = deepcopy(self.start_pose)
            pose.position.x = end[0]
            pose.position.y = end[1]
            pose.position.z = end[2]
            waypoints.append(deepcopy(pose))
        
        fraction = 0.0   # 路径规划覆盖率
        maxtries = 100   # 最大尝试规划次数
        attempts = 0     # 已经尝试规划次数
            
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
     
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
                
            # 尝试次数累加
            attempts += 1
                
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.arm.execute(plan, wait=True)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
