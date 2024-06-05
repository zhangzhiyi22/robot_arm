import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point
from copy import deepcopy
import math
from visualization_msgs.msg import Marker


class MoveItConeDemo:
    def __init__(self):
        # Initialize move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize ROS node
        rospy.init_node('moveit_cone_demo', anonymous=True)
        
        # Initialize MoveGroupCommander for the arm
        self.arm = MoveGroupCommander('arm')
        
        # Allow replanning when motion planning fails
        self.arm.allow_replanning(True)
        
        # Set the reference frame for target poses
        self.arm.set_pose_reference_frame('base_link')
                
        # Set position (unit: meter) and orientation (unit: radian) tolerance
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        # Set maximum velocity and acceleration scaling factors
        self.arm.set_max_velocity_scaling_factor(0.1) 
        self.arm.set_max_acceleration_scaling_factor(0.001)
        
        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()
                                        
        # Move the arm to the "home" position
        self.arm.set_named_target('home')
        self.arm.go()
        
        # Get the current pose of the end-effector
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                
        # Initialize waypoints list
        self.waypoints = []

        # Define the apex of the cone
        apex = deepcopy(start_pose)
        apex.position.z += 0.2  # Move up by 0.2 meters

        # Define the circle points
        radius = 0.1  # Radius of the circle at the base of the cone
        num_circle_points = 20  # Increased number of points on the circle for smoother path
        circle_points = []

        for i in range(num_circle_points):
            angle = 2 * math.pi * i / num_circle_points
            point = deepcopy(start_pose)
            point.position.x += radius * math.cos(angle)
            point.position.y += radius * math.sin(angle)
            circle_points.append(point)

        # Add lines from apex to every sixth circle point
        for i in range(0, num_circle_points, num_circle_points // 6):
            self.waypoints.append(deepcopy(apex))
            self.waypoints.append(deepcopy(circle_points[i]))

        # Add lines between circle points to form the base
        self.waypoints.extend(deepcopy(circle_points))

        # Visualization marker publisher
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.sleep(1)  # Give time for the publisher to initialize
        self.publish_cone_markers(apex, circle_points)

        # Wait for a moment to visualize the markers
        rospy.loginfo("Markers published. Waiting for visualization...")
        rospy.sleep(5)  # Adjust the sleep time as needed

        # Now start planning and executing the path
        self.plan_and_execute_path()

    def publish_cone_markers(self, apex, circle_points):
        """Publish markers to visualize the cone."""
        marker_id = 0

        # Line strip from apex to every sixth circle point
        for i in range(0, len(circle_points), len(circle_points) // 6):
            line_marker = self.create_line_strip_marker(marker_id, [apex.position, circle_points[i].position])
            self.marker_pub.publish(line_marker)
            marker_id += 1

        # Line strip to form the base circle
        base_circle_marker = self.create_line_strip_marker(marker_id, [p.position for p in circle_points] + [circle_points[0].position])
        self.marker_pub.publish(base_circle_marker)

    def create_marker(self, marker_id, position, marker_type):
        """Create a visualization marker."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker

    def create_line_strip_marker(self, marker_id, points):
        """Create a line strip marker to visualize lines."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Line width
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = [Point(p.x, p.y, p.z) for p in points]
        return marker
   

    



    def plan_and_execute_path(self):
        """Plan and execute the Cartesian path."""
        fraction = 0.0   # Path planning coverage rate
        maxtries = 100   # Maximum number of attempts
        attempts = 0     # Number of attempts

        # Set the current state as the initial state for motion
        self.arm.set_start_state_to_current_state()

        # Initialize a list to store waypoints that cause issues
        problem_waypoints = []

        # Try to plan a Cartesian path through all waypoints
        while fraction < 1.0 and attempts < maxtries:
            try:
                (plan, fraction) = self.arm.compute_cartesian_path(
                                        self.waypoints,   # waypoint poses list
                                        0.01,        # eef_step, increased step size
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions

                # Check and fix time_from_start issues
                plan = self.fix_trajectory_time(plan)

                # If path planning is successful, move the arm
                if fraction == 1.0:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    self.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                    break
            except Exception as e:
                rospy.logwarn(f"Exception during path planning: {str(e)}")
                problem_waypoints.append((self.waypoints[attempts], str(e)))
            
            # Increment attempt counter
            attempts += 1
            
            # Print progress
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        if fraction < 1.0:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        # Log problem waypoints
        if problem_waypoints:
            rospy.loginfo("Problematic waypoints:")
            for wp, err in problem_waypoints:
                rospy.loginfo(f"Waypoint: {wp}, Error: {err}")

        # Move the arm back to the "home" position
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
        # Shut down MoveIt and exit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def fix_trajectory_time(self, plan):
        """Fix trajectory time_from_start to ensure strictly increasing time."""
        previous_time = rospy.Duration(0.0)
        for point in plan.joint_trajectory.points:
            if point.time_from_start <= previous_time:
                point.time_from_start = previous_time + rospy.Duration(0.01)  # Increment by a small amount
            previous_time = point.time_from_start
        return plan

if __name__ == "__main__":
    try:
        MoveItConeDemo()
    except rospy.ROSInterruptException:
        pass
