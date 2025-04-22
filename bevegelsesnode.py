#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String

class BevegelsesNode(Node):
    def __init__(self):
        super().__init__('bevegelsesnode')
        self.sub = self.create_subscription(String, 'bryter/status', self.status_callback, 10)
        self.pub = self.create_publisher(Int16MultiArray, 'bevegelses/kommando', 10)
        self.timer_pirouette = None
        self.get_logger().info("Bevegelsesnode startet og venter på bryter‐endringer.")

    def status_callback(self, msg: String):
        if msg.data == 'ON':
            self.get_logger().info("Bryter ON oppdaget – starter pirouette om 1 sekund.")
            # Vent 1 s før pirouette
            if self.timer_pirouette:
                self.timer_pirouette.cancel()
            self.timer_pirouette = self.create_timer(1.0, self.pirouette)
        else:
            self.get_logger().info("Bryter OFF – stopper bevegelse.")
            # send stopp‐kommando umiddelbart
            cmd = Int16MultiArray()
            cmd.data = [0, 0]
            self.pub.publish(cmd)
    def pirouette(self):
        # Én kjøring – så avbestill timer
        if self.timer_pirouette:
            self.timer_pirouette.cancel()
            self.timer_pirouette = None

        # Sett hjul‐hastigheter: venstre = +50, høyre = -50
        cmd = Int16MultiArray()
        cmd.data = [50, -50]
        self.pub.publish(cmd)
        self.get_logger().info("Pirouette pågår – wheel speeds [50, -50]")

        # Etter 3 s, send stopp
        self.create_timer(3.0, self.stop_motors, callback_group=None)

    def stop_motors(self):
        cmd = Int16MultiArray()
        cmd.data = [0, 0]
        self.pub.publish(cmd)
        self.get_logger().info("Pirouette ferdig – stopper hjulene.")

def main():
    rclpy.init()
    node = BevegelsesNode()
    try:
        rclpy.spin(node)
	pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
