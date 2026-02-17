#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import json

# Topic names: cv_camera, telem, camera, targets, bbox

class StichingNode(Node):

    def __init__(self):
        super().__init__('example_node')

        # -------------------------
        # Publisher (does )
        # -------------------------

        # -------------------------
        # Subscriber
        # -------------------------
        self.subscription = self.create_subscription(
            String,
            'cv_camera',
            self.subscriber_callback,
            10
        )

        # -------------------------
        # Timer (runs every 1 second)
        # -------------------------
        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.get_logger().info('Example Node has started.')

    # ==========================================================
    # Subscriber Callback
    # ==========================================================
    def subscriber_callback(self, msg):
        """
        Called whenever a message is received on 'input_topic'.
        """
        self.get_logger().info(f"Received: {msg.data}")

        try:
            data = json.loads(msg.data)

            # Example: state transition trigger
            if data.get("command") == "start":
                self.transition_to(ExampleState.RUNNING)

            elif data.get("command") == "stop":
                self.transition_to(ExampleState.IDLE)

        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON")

    # ==========================================================
    # Timer Callback
    # ==========================================================
    def timer_callback(self):
        """
        Runs periodically (based on timer frequency).
        Good for status publishing, health checks, etc.
        """
        if self.state == ExampleState.RUNNING:
            self.publish_status("Node is running...")

    # ==========================================================
    # Publisher Helper
    # ==========================================================
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {text}")

    # ==========================================================
    # State Machine Handling
    # ==========================================================
    # def transition_to(self, new_state: ExampleState):

    # def on_enter_state(self, state: ExampleState):

class CameraNode(Node):

    def __init__(self):
        super().__init__('example_node')

        # -------------------------
        # State Machine
        # -------------------------

        # -------------------------
        # Publisher
        # -------------------------
        self.publisher_ = self.create_publisher(
            String,
            'cv_camera',
            10
        )

        # -------------------------
        # Subscriber
        # -------------------------
        self.subscription = self.create_subscription(
            String,
            'telem',
            self.subscriber_callback,
            10
        )

        self.subscription = self.create_subscription(
            String,
            'camera',
            self.subscriber_callback,
            10
        )

        # -------------------------
        # Timer (runs every 1 second)
        # -------------------------
        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.get_logger().info('Example Node has started.')

    # ==========================================================
    # Subscriber Callback
    # ==========================================================
    def subscriber_callback(self, msg):
        """
        Called whenever a message is received on 'input_topic'.
        """
        self.get_logger().info(f"Received: {msg.data}")

        try:
            data = json.loads(msg.data)

        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON")

    # ==========================================================
    # Timer Callback
    # ==========================================================
    def timer_callback(self):
        """
        Runs periodically (based on timer frequency).
        Good for status publishing, health checks, etc.
        """

    # ==========================================================
    # Publisher Helper
    # ==========================================================
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {text}")

    # ==========================================================
    # State Machine Handling
    # ==========================================================
    # def transition_to(self, new_state: ExampleState):
    
    # def on_enter_state(self, state: ExampleState):

class ODLCNode(Node):

    def __init__(self):
        super().__init__('example_node')

        # -------------------------
        # State Machine
        # -------------------------

        # -------------------------
        # Publisher
        # -------------------------
        self.publisher_ = self.create_publisher(
            String,
            'targets',
            10
        )

        self.publisher_ = self.create_publisher(
            String,
            'bbox',
            10
        )

        # -------------------------
        # Subscriber
        # -------------------------
        self.subscription = self.create_subscription(
            String,
            'cv_camera',
            self.subscriber_callback,
            10
        )

        # -------------------------
        # Timer (runs every 1 second)
        # -------------------------
        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.get_logger().info('Example Node has started.')

    # ==========================================================
    # Subscriber Callback
    # ==========================================================
    def subscriber_callback(self, msg):
        """
        Called whenever a message is received on 'input_topic'.
        """
        self.get_logger().info(f"Received: {msg.data}")

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON")

    # ==========================================================
    # Timer Callback
    # ==========================================================
    def timer_callback(self):
        """
        Runs periodically (based on timer frequency).
        Good for status publishing, health checks, etc.
        """

    # ==========================================================
    # Publisher Helper
    # ==========================================================
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {text}")

    # ==========================================================
    # State Machine Handling
    # ==========================================================
    # def transition_to(self, new_state: ExampleState):

    # def on_enter_state(self, state: ExampleState):

# ==========================================================
# Main Entry Point
# ==========================================================
def main(args=None):
    rclpy.init(args=args)

    node = CameraNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()