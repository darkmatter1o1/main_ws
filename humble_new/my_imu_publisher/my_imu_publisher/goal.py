#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Float32



class Nav2GoalSequencer(Node):
    def __init__(self):
        super().__init__('goal_sequencer')

        # Subscribers
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(Int32, '/selected_goal1', self.goal_sequence_callback, 10)
        self.create_subscription(Float32, '/battery_level', self.battery_callback, 10)  # Battery subscriber
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)


        # Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # States
        self.goal_queue = []
        self.current_goal = None
        self.current_goal_id = None  # NEW: remember current goal ID
        self.pause = False
        self.low_battery = False
        self.retry_flag = False
        self.sequence_running = False
        self.delay_between_goals = .3  # seconds
        self.resume_delay_timer = None  # NEW: battery resume timer
        self.timer = None  # Goal scheduling timer

        # Define your goals (ID: Pose)
        self.goals = {
            0: self.make_pose_q(7.21, 7.24, 0.00, 0.99),
            1: self.make_pose_q(8.14, 7.43, -0.65, 0.75),
            2: self.make_pose_q(8.22, 5.93, -0.03, 0.99),
            3: self.make_pose_q(11.70, 6.23, 0.62, 0.77),
            4: self.make_pose_q(11.45, 8.50, -0.57, 0.81),
            5: self.make_pose_q(12.23, 2.64, -0.99, 0.11),
            6: self.make_pose_q(7.36, 2.15, 0.67, 0.74)
        }
        self.sequences = {
            1: [0, 1, 2, 3, 4, 5, 6],
            2: [0, 4, 1],
            3: [1, 3, 4, 7]
        }

    def make_pose_q(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def battery_callback(self, msg: Int32):
        battery = msg.data
        if battery < 10 and not self.low_battery:
            self.low_battery = True
            self.get_logger().warn(f"🔋 Low battery ({battery}%). Pausing navigation...")
            self.cancel_current_goal()

        elif battery > 35 and self.low_battery:
            self.low_battery = False
            self.get_logger().info(f"🔋 Battery recovered ({battery}%). Waiting 60s before resuming...")
            if self.resume_delay_timer:
                self.resume_delay_timer.cancel()
            self.resume_delay_timer = self.create_timer(
                60.0,  # wait 60 sec before resume
                self._resume_after_battery_timer
            )

    def _resume_after_battery_timer(self):
        self.resume_delay_timer.cancel()
        self.resume_delay_timer = None
        self.resume_sequence_after_battery()

    def obstacle_callback(self, msg):
        if msg.data:
            self.pause = True
            self.cancel_current_goal()
        else:
            if self.pause and self.current_goal is not None:
                self.get_logger().info("Retrying last canceled goal due to obstacle...")
                self.pause = False
                self.retry_flag = True
                self.send_goal(self.current_goal_id)

    
    # Add in __init__ right after your other subscribers

# Add the callback method
    def emergency_stop_callback(self, msg):
        if msg.data:  # Emergency stop triggered
            self.get_logger().error("🛑 EMERGENCY STOP activated! Cancelling navigation and clearing queue.")
            self.cancel_current_goal()
            self.goal_queue.clear()
            self.current_goal = None
            self.current_goal_id = None
            self.sequence_running = False
            self.pause = False
            self.low_battery = False
        # Cancel timers if any
            if self.timer:
                self.timer.cancel()
                self.timer = None
            if self.resume_delay_timer:
                self.resume_delay_timer.cancel()
                self.resume_delay_timer = None


    def goal_sequence_callback(self, msg):
        if self.sequence_running:
            self.get_logger().warn("⚠️ A sequence is already running. Ignoring new request.")
            return

        if msg.data in self.sequences:
            self.goal_queue = self.sequences[msg.data][:]
            self.get_logger().info(f"📌 Sequence {msg.data} received.")
            self.sequence_running = True
            self.process_goal_sequence()
        else:
            self.get_logger().warn(f"❓ Unknown sequence ID: {msg.data}")

    def send_goal(self, goal_id):
        if self.low_battery:
            self.get_logger().warn("⚠️ Cannot send goal — battery is low.")
            return

        pose = self.goals.get(goal_id)
        if pose is None:
            self.get_logger().error(f"❌ Goal ID {goal_id} not found.")
            return

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        self.current_goal = pose
        self.current_goal_id = goal_id  # Remember current goal ID

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"🚀 Sending goal ID={goal_id} (x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f})")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('❌ Goal was rejected!')
            return
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("✅ Goal reached.")
        else:
            self.get_logger().warn(f"⚠️ Goal failed with error code: {result.error_code}")

        if not self.pause and not self.low_battery:
            self.schedule_next_goal()
        self.retry_flag = False

    def cancel_current_goal(self):
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self.get_logger().info("🚫 Goal canceled.")

    def schedule_next_goal(self):
        if self.low_battery:
            self.get_logger().warn("⚠️ Next goal postponed due to low battery.")
            return

        if self.goal_queue:
            self.get_logger().info(f"⏳ Waiting {self.delay_between_goals} sec before next goal...")
            if self.timer:
                self.timer.cancel()
            self.timer = self.create_timer(
                self.delay_between_goals,
                self._next_goal_timer_callback
            )
        else:
            self.get_logger().info("🎯 All goals completed.")
            self.sequence_running = False
            self.current_goal = None
            self.current_goal_id = None

    def _next_goal_timer_callback(self):
        self.timer.cancel()
        self.timer = None
        next_id = self.goal_queue.pop(0)
        self.send_goal(next_id)

    def process_goal_sequence(self):
        self.schedule_next_goal()

    def resume_sequence_after_battery(self):
        if self.current_goal_id is not None:
            self.get_logger().info(f"🔄 Resuming from goal ID={self.current_goal_id}")
            self.send_goal(self.current_goal_id)
        elif self.goal_queue:
            self.get_logger().info("🔄 Resuming from next goal in queue")
            self.schedule_next_goal()
        else:
            self.get_logger().info("✅ No goals left to resume.")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSequencer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
