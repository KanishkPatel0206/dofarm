#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class SquareTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('square_traj_publisher')
        self.pub_j1 = self.create_publisher(Float64, '/model/dof_arm/joint1/cmd_pos', 10)
        self.pub_j2 = self.create_publisher(Float64, '/model/dof_arm/joint2/cmd_pos', 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.l1 = 0.20
        self.l2 = 0.20 
        self.max_reach = self.l1 + self.l2

        self.L = 0.15
        self.cx = 0.20
        self.cy = 0.0
        self.speed = 0.05

        self.corners = [
            (self.cx - self.L/2, self.cy - self.L/2),
            (self.cx + self.L/2, self.cy - self.L/2),
            (self.cx + self.L/2, self.cy + self.L/2),
            (self.cx - self.L/2, self.cy + self.L/2),
        ]
        self.corners = [self.clamp_to_workspace(p) for p in self.corners]

        self.segment = 0
        self.seg_start_time = time.time()

        self.q_prev = (0.0, 0.0)

        self.elbow = 1

        self.alpha = 0.25

    def clamp_to_workspace(self, point):
        x, y = point
        r = math.hypot(x, y)
        if r > 0.95 * self.max_reach:
            s = (0.95 * self.max_reach) / r
            return (x * s, y * s)
        return point

    def smooth(self, q, q_prev):
        return (
            self.alpha * q[0] + (1 - self.alpha) * q_prev[0],
            self.alpha * q[1] + (1 - self.alpha) * q_prev[1],
        )


    def inverse_kinematics(self, x, y):
        d = (x*x + y*y - self.l1*self.l1 - self.l2*self.l2) / (2*self.l1*self.l2)
        d = max(min(d, 1.0), -1.0) 

        q2 = self.elbow * math.acos(d)

        q1 = math.atan2(y, x) - math.atan2(
            self.l2 * math.sin(q2),
            self.l1 + self.l2 * math.cos(q2)
        )

        return q1, q2


    def timer_callback(self):
        t = time.time()

        p1 = self.corners[self.segment]
        p2 = self.corners[(self.segment + 1) % len(self.corners)]

        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        duration = dist / self.speed
        u = min((t - self.seg_start_time) / duration, 1.0)

        x = p1[0] + u * (p2[0] - p1[0])
        y = p1[1] + u * (p2[1] - p1[1])

        if u >= 1.0:
            self.segment = (self.segment + 1) % len(self.corners)
            self.seg_start_time = t

        q = self.inverse_kinematics(x, y)

        q = self.smooth(q, self.q_prev)
        self.q_prev = q

        msg1 = Float64()
        msg1.data = float(q[0])
        
        msg2 = Float64()
        msg2.data = float(q[1])
        
        self.pub_j1.publish(msg1)
        self.pub_j2.publish(msg2)
        
def main(args=None):
    rclpy.init(args=args)
    node = SquareTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
