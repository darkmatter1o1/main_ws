import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

class MultiPoseNav(Node):
    def __init__(self):
        super().__init__('multi_pose_nav')
        self._client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.send_goals()

    def send_goals(self):
        # wait for server
        self._client.wait_for_server()

        # define poses
        poses = []
        points = [(1.0,0.0),(2.0,0.0),(3.0,0.0)]  # your straight line points
        for i in range(len(points)-1):
            x, y = points[i]
            next_x, next_y = points[i+1]
            yaw = math.atan2(next_y - y, next_x - x)
            q = quaternion_from_euler(0,0,yaw)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            poses.append(pose)

        # include last point
        last_x, last_y = points[-1]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = last_x
        pose.pose.position.y = last_y
        pose.pose.orientation.w = 1.0
        poses.append(pose)

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self._client.send_goal_async(goal_msg)
        self.get_logger().info("Goal sent!")

rclpy.init()
node = MultiPoseNav()
rclpy.spin(node)
