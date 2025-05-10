# VERY weak algorihtm
# CAN be tuned to the maze
#
# Tuning Parameters
#  1. Commitment Clocks
<<<<<<< Updated upstream

import math
=======
#  2. Drift Speed (Efficiency)
#  3. Turning and Movement Speed
>>>>>>> Stashed changes
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import math

import collections

class CollisionBox(collections.namedtuple('CBox', ['hw', 'l'])):
    """ Collision Box
    Represents a virtual collision box at the front of the robot.
    Has a half-width and a length
    """
    __slots__ = ()

# Don't publish over 0.6 to motors.
def frac_to_speed(s: float):
    if s > 1.1: print("Bad Input to Speed"); exit(1)
    elif s > 1.: print("Warn Overclocking Motors")
    return s * 0.6

class RobotController(Node):
    """
        Robot Movement Controller
        
        Tuning Parameters
         1. Commitment Clocks
         2. Drift Speed         (Efficiency)
         3. Turn Speed          (Winning)
         3. Movement Speed      (Winning)
    """
    def __init__(self):
        super().__init__("h1_robot_movement_controller")
       
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)
       
        self.seconds_per_clock = 0.25
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(self.seconds_per_clock, self.set_velocity)
        
        # Settings
        self.drift = 0. # 1./3
        self.stop = 0

        # NOTE: To convert to fractional units
        # Idea is to base this on velocity
        self.cbox = CollisionBox(hw=0.3, l=0.4)

        self.turn_velocity_frac = 1.0
        self.slower_turn_velocity_frac = 0.5
        self.forward_velocity_frac = 1.0 # Driver speeds not accurate when large

        # Noise Tolerance Controls
        self.sensor_size = 20

        # Toggle Controllers
        self.toggle = False
        self.lpub = False
        self.rpub = False

        self.blpub = False

        self.i = 0
        
        # Timed Controllers
        self.commit_clocks = 0
        self.commit_clocks_max = 100 # clocks

    def pop_fill(self, msg, i, i_arr, dist_arr):
        """ Fill sensing array O(self.sensor_size) """
        while len(dist_arr) < self.sensor_size:
            if (msg.ranges[i] < msg.range_max 
                    and msg.ranges[i] > 1.1):
                dist_arr.append(msg.ranges[i])
                i_arr.append(i)

            i += 1
            i %= len(msg.ranges)

        return False

    def sense(self, msg):
        """
            Denoise sensor outputs T(res) = self.sensor_size * len(msg.ranges)
            Possible to add specialized kernels to detect features here
        """
        noise_filter_i = []
        noise_filter_dist = []

        contender_filter_i = []
        contender_filter_dist = []

        # Rangemax
        i = 0
        self.pop_fill(msg, i, noise_filter_i, noise_filter_dist)

        x = len(msg.ranges)//7
        for j in range(len(msg.ranges)):
            i = j
            for k, d in zip(noise_filter_i, noise_filter_dist):
                if (msg.ranges[i] > d) and (msg.ranges[i] < msg.range_max):
                    overflowed = self.pop_fill(msg, i, contender_filter_i, contender_filter_dist)
                    if (sum(noise_filter_dist)/len(noise_filter_dist) < sum(contender_filter_dist)/len(contender_filter_dist)):
                        noise_filter_dist = contender_filter_dist
                        noise_filter_i = contender_filter_i
                    contender_filter_dist = []
                    contender_filter_i = []
   
        sensor_position = sum(noise_filter_i)/len(noise_filter_i)
        sensor_dist = sum(noise_filter_dist)/len(noise_filter_dist) 

        return (sensor_dist, sensor_position)   


    def navigate(self,msg):
<<<<<<< Updated upstream
=======
        """ Points robot at furthest unknown. """
>>>>>>> Stashed changes
        (sensor_dist, sensor_position) = self.sense(msg)

        bias_cutoff = len(msg.ranges)//2

        if sensor_position - bias_cutoff > 0: self.rpub = True
        elif sensor_position - bias_cutoff < 0: self.lpub = True


    def obstacle(self,msg):
        """
            Robot Sensor Controller/Driver
            Operates at speed of sensor for the best problem resolution
        """
        x = len(msg.ranges)//7

        # # Hardstop with Safety Box (Never toggled)
        # #angle = ((7 - len(msg.ranges)//2) / len(msg.ranges)) * 2 * math.pi / 2 # Compiler opt
        # angle = (sensor_position / len(msg.ranges) - 0.5) * math.pi
        # ydist = math.sin(angle) * sensor_dist
        # axdist = abs(math.cos(angle) * sensor_dist)

        # Toggle Controllers
        # collider_hit = axdist > self.cbox.hw and ydist < self.cbox.l and ydist > 0
        maybe_open_ahead = True
        for j in range(x):
            i = j + 3*x
            # If one point in the range is worse than the act_dist 
            # (Then we're committing)
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] < msg.range_max):
                maybe_open_ahead = False
                break
        
        if not maybe_open_ahead:
            # Look for a better distance
            self.stop = 1
            self.commit_clocks = 0 # Reset commitment clock

            self.navigate(msg)
        else:
            # Commit
            self.lpub = False
            self.rpub = False
            self.stop = 0

            # Commitment Clock
            self.commit_clocks = (self.commit_clocks + 1) % self.commit_clocks_max
            if self.commit_clocks == 0:
                self.navigate(msg)
    
    def set_velocity(self):
        """ Robot Motor Controller """

        msg = Twist()
        if self.stop == 0:
           msg.linear.x = frac_to_speed(self.forward_velocity_frac)
           self.get_logger().info('Moving')
        else:
           msg.linear.x = .0
           self.get_logger().info('Stopping')


        # NOTE: In testing turn Messages overwrite previous messages
        # # set linear velocity
        self.velocity_.publish(msg)
        # Effectively Elif
        # Using drift to clarify

        if self.lpub:
            self.turnL()
            self.lpub = False
            self.get_logger().info('Turning L')
        elif self.rpub:
            self.turnR()
            self.rpub = False
            self.get_logger().info('Turning R')
        elif self.stop:
            self.slow_release()
            self.get_logger().info('Releasing')
        
        self.turn_timer_slow += self.seconds_per_clock
        self.turn_timer_slow %= self.turn_timer_slow_max
    

    # Make sure robot does not get stuck
    def slow_release(self):
        """
            The slow release allows a robot to get unstuck
            if a collider is hit and the base algorithm has
            no good answer.
        """
        msg = Twist()
        msg.linear.x = frac_to_speed(self.forward_velocity_frac)
        msg.angular.z = self.slower_turn_velocity_frac * (-1 if self.blpub else 1)
        self.velocity_.publish(msg)

    def turnL(self):
        """ Turn robot Counterclockwise """
        msg = Twist()
        msg.angular.z = -frac_to_speed(self.turn_velocity_frac)
        if self.stop == 0: msg.linear.x = self.drift # Speed up robot around corners
        self.velocity_.publish(msg)

    def turnR(self):
        """ Turn robot Clockwise """
        msg = Twist()
        msg.angular.z = frac_to_speed(self.turn_velocity_frac)
        if self.stop == 0: msg.linear.x = self.drift # Speed up robot around corners
        self.velocity_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    node = RobotController()

    # Run the node
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
