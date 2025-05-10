# This implementation moves around a wall maze.
# If there is no block to either side the robot
# wraps in a calibrated around the last block it saw.
# 
# Visual
#     [?]
# [][] } Calibrated arc
# [] r-
# [] |

from time import sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy

class RobotController(Node):
    """
        Robot Movement Controller
        
        Tuning Parameters
         1. Follow Right or Left Wall (self.lwall_follow)
         2. Actuation distance

        Embedded Tuning Parameters
         1. Slowly arc right (Tuned to open spaces in maze)
         2. Quickly turn away from wall (Tuned to wall thickness of maze)
    """
    def __init__(self):
        super().__init__("wallfollow_robot_controller")

        # Sub to sensor
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)

        # Controllers
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.set_velocity) # rate of cmds
        
        # Calibration settings
        self.act_dist = 0.38 
        self.lwall_follow = True

        # Toggles
        self.stop = 0
        self.lpub = False
        self.rpub = False
        self.slpub = False
        self.srpub = False # Unused
        self.turn_timer_slow = 0
        self.turn_timer_slow_max = 2 # seconds

    def navigate(self,msg):
        """ Keeps robot at wall """
        self.get_logger().info('Turning')
        x = len(msg.ranges) // 10

        lwall = False
        rwall = False
        # Left
        for j in range(int(2*x)): # NOTE: this sensor is slightly larger!
            i = j + 5*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                lwall = True
                self.get_logger().info('Left wall detected')

        # Right
        for j in range(2*x):
            i = j + 3*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                rwall = True
                self.get_logger().info('Right wall detected')
        
        # Engage motor actions     
        if not rwall and not lwall: 
            if self.lwall_follow:   self.slpub = True
            else:                   self.srpub = True
            self.get_logger().info('No detection')
        elif rwall: self.lpub = True
        elif lwall: self.rpub = True
        else:
            # Stop rotation here (No timer callback)
            msg = Twist()
            msg.angular.z = .0
            self.velocity_.publish(msg)

    def obstacle(self,msg):
        """
            Robot Sensor Controller/Driver
            Operates at speed of sensor for the best problem resolution
        """
        x = len(msg.ranges)//8

        # Front
        for j in range(2*x):
            i = j + 3*x
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                self.stop = 1
                break
            else:
                self.stop = 0
    
        self.navigate(msg)


    def set_velocity(self):
        """ Robot Motor Controller """

        msg = Twist()
        if (self.stop == 0):
           msg.linear.x = .075
           self.get_logger().info('Moving')
        else:
           msg.linear.x = .0
           self.get_logger().info('Stopping')

        # set linear velocity
        self.velocity_.publish(msg)
        
        # Turn timers slow speed of arc reliably
        # Easier for humans to tune.

        if self.turn_timer_fast % self.turn_timer_fast_max == 0:
            if self.rpub and self.lwall_follow:
                self.turnR()
                self.rpub = False
                self.get_logger().info('Quick Turning R')
            elif self.lpub:
                self.turnL()
                self.lpub = False
                self.get_logger().info('Quick Turning L')
            elif self.rpub:
                self.turnR()
                self.rpub = False
                self.get_logger().info('Quick Turning R')
        
        if self.turn_timer_slow % self.turn_timer_slow_max == 0:
            if self.slpub:
                self.turnL()
                self.slpub = False
                self.get_logger().info('Turning L')
            elif self.srpub:
                self.turnR()
                self.srpub = False
                self.get_logger().info('Turning R')
        
        self.turn_timer_fast = self.seconds_per_clock
        self.turn_timer_slow += self.seconds_per_clock

        self.turn_timer_fast %= self.turn_timer_fast_max
        self.turn_timer_slow %= self.turn_timer_slow_max
        

    def turnL(self):
        """ Turn robot Counterclockwise """
        msg = Twist()
        msg.angular.z = 0.25 # Calibrate the arc (lwall_follow)
        self.velocity_.publish(msg)

    def turnR(self):
        """ Turn robot Clockwise """
        msg = Twist()
        msg.angular.z = -0.25 # Good, speed of rotate away from opposing obstacle
        self.velocity_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

