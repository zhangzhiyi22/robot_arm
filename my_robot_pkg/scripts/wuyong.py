import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point
from copy import deepcopy
import math
from visualization_msgs.msg import Marker

class MoveItFigure8Demo:
    def __init__(self):
        # Initialize move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize ROS node
        rospy.init_node('moveit_figure8_demo', anonymous=True)
        
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
        self.arm.set_max_acceleration_scaling_factor(0.05)
        
        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()
                                        
        # Move the arm to the "home" position
        self.arm.set_named_target('home')
        self.arm.go()
        
        # Get the current pose of the end-effector
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                
        # Initialize waypoints list
        self.waypoints = []

        # Define the radius and number of points
        radius = 0.1
        num_points = 50  # More points for smoother figure-8

        # Create the figure-8 paths in XY, XZ, and YZ planes
        xy_points = self.create_figure8_path(start_pose, radius, num_points, plane='xy')
        xz_points = self.create_figure8_path(start_pose, radius, num_points, plane='xz')
        yz_points = self.create_figure8_path(start_pose, radius, num_points, plane='yz')

        # Combine all points into waypoints
        self.waypoints.extend(xy_points)
        self.waypoints.extend(xz_points)
        self.waypoints.extend(yz_points)

        # Visualization marker publisher
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.sleep(1)  # Give time for the publisher to initialize
        self.publish_markers(xy_points, xz_points, yz_points)

        # Wait for a moment to visualize the markers
        rospy.loginfo("Markers published. Waiting for visualization...")
        rospy.sleep(5)  # Adjust the sleep time as needed

        # Now start planning and executing the path
        self.plan_and_execute_path()

    def create_figure8_path(self, start_pose, radius, num_points, plane='xy'):
        """Create a figure-8 path in the specified plane."""
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            point = deepcopy(start_pose)
            if plane == 'xy':
                point.position.x += radius * math.cos(angle)
                point.position.y += radius * math.sin(2 * angle) / 2
            elif plane == 'xz':
                point.position.x += radius * math.cos(angle)
                point.position.z += radius * math.sin(2 * angle) / 2
            elif plane == 'yz':
                point.position.y += radius * math.cos(angle)
                point.position.z += radius * math.sin(2 * angle) / 2
            points.append(point)
        return points

    def publish_markers(self, xy_points, xz_points, yz_points):
        """Publish markers for waypoints to visualize the path."""
        marker_id = 0

        # Publish markers for XY plane points
        self.publish_line_strip_marker(marker_id, xy_points)
        marker_id += 1

        # Publish markers for XZ plane points
        self.publish_line_strip_marker(marker_id, xz_points)
        marker_id += 1

        # Publish markers for YZ plane points
        self.publish_line_strip_marker(marker_id, yz_points)

    def publish_line_strip_marker(self, marker_id, points):
        """Create and publish a line strip marker."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "figure8"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Line width, adjust this value to change the thickness
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = [Point(p.position.x, p.position.y, p.position.z) for p in points]
        self.marker_pub.publish(marker)

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
        MoveItFigure8Demo()
    except rospy.ROSInterruptException:
        pass
