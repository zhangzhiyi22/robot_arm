#!/usr/bin/env python3

import rospy
import time
import numpy as np
from kortex_driver.srv import *
from kortex_driver.msg import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point




class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            # Initialize the marker publisher
            self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_send_pose(self, pose, identifier, name):
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = 0.1  # m/s
        my_cartesian_speed.orientation = 15  # deg/s

        my_constrained_pose = ConstrainedPose()
        my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

        my_constrained_pose.target_pose.x = pose[0]
        my_constrained_pose.target_pose.y = pose[1]
        my_constrained_pose.target_pose.z = pose[2]
        my_constrained_pose.target_pose.theta_x = pose[3]
        my_constrained_pose.target_pose.theta_y = pose[4]
        my_constrained_pose.target_pose.theta_z = pose[5]

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
        req.input.name = name
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = identifier

        rospy.loginfo(f"Sending pose {name}...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr(f"Failed to send pose {name}")
            return False
        else:
            rospy.loginfo(f"Waiting for pose {name} to finish...")
            return self.wait_for_action_end_or_abort()

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

    def main(self):
        success = self.is_init_success

        if success:
            success &= self.example_clear_faults()

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

           

            # # 定义正方体的顶点，基于home位置(0.435, 0.194, 0.457)
            # vertices = [
            #     [0.435, 0.194, 0.457],             # 顶点 1 (home)
            #     [0.535, 0.194, 0.457],             # 顶点 2 (沿X轴正方向)
            #     [0.535, 0.294, 0.457],             # 顶点 3 (沿X和Y轴正方向)
            #     [0.435, 0.294, 0.457],             # 顶点 4 (沿Y轴正方向)
            #     [0.435, 0.194, 0.557],             # 顶点 5 (沿Z轴正方向)
            #     [0.535, 0.194, 0.557],             # 顶点 6 (沿X和Z轴正方向)
            #     [0.535, 0.294, 0.557],             # 顶点 7 (沿X、Y和Z轴正方向)
            #     [0.435, 0.294, 0.557]              # 顶点 8 (沿Y和Z轴正方向)
            # ]

            # # 定义正方体的边
            # edges = [
            #     (0, 1), (1, 2), (2, 3), (3, 0),  # 底面四条边
            #     (4, 5), (5, 6), (6, 7), (7, 4),  # 顶面四条边
            #     (0, 4), (1, 5), (2, 6), (3, 7)   # 竖直方向四条边
            # ]

     

            # 生成插值点
            num_points_per_edge = 10
            interpolated_poses = []

            for edge in edges:
                start, end = vertices[edge[0]], vertices[edge[1]]
                for t in np.linspace(0, 1, num_points_per_edge):
                    interp_pose = [
                        start[i] * (1 - t) + end[i] * t
                        for i in range(3)
                    ] + [90, 0, 150]  # 保持角度不变
                    interpolated_poses.append(interp_pose)

            # 发布Marker显示轨迹
            self.publish_marker(interpolated_poses)

            # 发送插值点
            for i, pose in enumerate(interpolated_poses):
                success &= self.example_send_pose(pose, 1001 + i, f"pose_{i + 1}")
                if not success:
                    break

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    ex.main()
