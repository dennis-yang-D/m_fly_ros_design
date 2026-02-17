#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import json


class ExampleState(Enum):
    IDLE = 0
    RUNNING = 1
    COMPLETE = 2


class ExampleNode(Node):

    def __init__(self):
        super().__init__('example_node')

        # -------------------------
        # State Machine
        # -------------------------
        self.state = ExampleState.IDLE
        self._entered_state_once = False

        # -------------------------
        # Publisher
        # -------------------------
        self.publisher_ = self.create_publisher(
            String,
            'example_topic',
            10
        )

        # -------------------------
        # Subscriber
        # -------------------------
        self.subscription = self.create_subscription(
            String,
            'input_topic',
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
    def transition_to(self, new_state: ExampleState):
        if new_state == self.state and self._entered_state_once:
            return

        self.get_logger().info(
            f"State change: {self.state.name} -> {new_state.name}"
        )

        self.state = new_state
        self._entered_state_once = False
        self.on_enter_state(new_state)

    def on_enter_state(self, state: ExampleState):
        """
        Actions that run once when entering a state.
        """
        if self._entered_state_once:
            return

        if state == ExampleState.IDLE:
            self.get_logger().info("Entered IDLE state.")

        elif state == ExampleState.RUNNING:
            self.get_logger().info("Entered RUNNING state.")

        elif state == ExampleState.COMPLETE:
            self.get_logger().info("Entered COMPLETE state.")

        self._entered_state_once = True


# ==========================================================
# Main Entry Point
# ==========================================================
def main(args=None):
    rclpy.init(args=args)

    node = ExampleNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()