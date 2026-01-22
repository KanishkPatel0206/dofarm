#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  
import time

class TrapTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trap_traj_publisher')

        self.pub_joint1 = self.create_publisher(Float64, '/model/dof_arm/joint1/cmd_pos', 10)
        self.pub_joint2 = self.create_publisher(Float64, '/model/dof_arm/joint2/cmd_pos', 10)
        
        self.start_time = time.time()
        self.timer = self.create_timer(0.03, self.timer_callback) 

        self.accel = [1.0, 1.0]           
        self.accel_time = [1.5, 1.0]      
        
        self.peak_vel = [
            self.accel[0] * self.accel_time[0], 
            self.accel[1] * self.accel_time[1]
        ]
        
        self.total_duration = 6.0         

    def trapezoidal_profile(self, t, joint_idx):
        v_peak = self.peak_vel[joint_idx]
        a = self.accel[joint_idx]
        t_acc = self.accel_time[joint_idx]
        
        if t < t_acc:
            return 0.5 * a * t**2

        t_decel_start = self.total_duration - t_acc
        
        if t < t_decel_start:
            pos_accel = 0.5 * a * t_acc**2
            return pos_accel + v_peak * (t - t_acc)

        if t < self.total_duration:
            t_dec = t - t_decel_start
            pos_accel = 0.5 * a * t_acc**2
            pos_const = v_peak * (t_decel_start - t_acc)
           
            return pos_accel + pos_const + (v_peak * t_dec - 0.5 * a * t_dec**2)

        t_mid = self.total_duration - 2*t_acc
        return (0.5 * a * t_acc**2) + (v_peak * t_mid) + (0.5 * a * t_acc**2)

    def timer_callback(self):
        t = time.time() - self.start_time
        
        if t > self.total_duration:
            t = self.total_duration

        q1 = self.trapezoidal_profile(t, 0)
        q2 = self.trapezoidal_profile(t, 1)

        msg1 = Float64()
        msg1.data = float(q1)
        
        msg2 = Float64()
        msg2.data = float(q2)

        self.pub_joint1.publish(msg1)
        self.pub_joint2.publish(msg2)
        
        self.get_logger().info(f'T: {t:.2f} | J1: {q1:.2f} | J2: {q2:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TrapTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
