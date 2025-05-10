# Perfect Heuristic Far execution
# Fastest approach
# Works well for SIMPLE disjoint mazes
# Where it does not work
#  ______________________________
#  | S                          |
#  |____________   _____________|
#              |  END >>
#              |______ 
# Robot would bounce back and forth
# Between the ends of the wall
# Where it's excellent
# ___________________________________
# |                                 |
# |______________________________   |
#  << END                           |
# __________________________________|
# This algorithm will not backtrack
# Unless confronted with a distracting
# event such as a more appealing unknown
# 
# Bennefit: Fastest heuristic approach 
#           for simple mazes
# Note:     Designed for square mazes

from time import sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np 

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
        super().__init__("number_publisher")

        # Sub to sensor
        self.Lidardis = self.create_subscription(LaserScan, "scan", self.obstacle, 10)

        # Controllers
        self.clock_speed = 0.5
        self.velocity_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(self.clock_speed, self.movement_callback) # rate of cmds
        
        # Calibration settings
        self.act_dist = 0.3
        self.timer_max = int(5 / self.clock_speed) # Units seconds to clocks
        self.old_heading = np.array([0, 0])
        self.old_rotation = np.array([0, 0])
        self.dir_bias = 0.4 # NOTE: if set to zero will never loop back on self for exploration

        self.live_dheading = 0
        self.live_drot = 0

        # Toggles
        self.timer = 1
        self.stop = 0
        self.lpub = False
        self.rpub = False

    # register far away points and store them
    # Heuristic way of tracking unknowns
    def movement_callback(self, msg):
        def pop_fill(i, i_arr, dist_arr, f=lambda x:x):
            nonlocal msg
            i = 0
            while len(dist_arr) < 8:
                if (msg.ranges[i] < msg.range_max):
                    dist_arr.append(msg.ranges[i])
                    i_arr.append(i)

                i += 1 # CAN ERR ...
                # Can buggie if dist too large! ...

            return False

        # Sensor collections
        noise_filter_i = []
        noise_filter_dist = []

        contender_filter_i = []
        contender_filter_dist = []

        # Rangemax
        i = 0
        self.pop_fill(i, noise_filter_i, noise_filter_dist)

        # Get the furthest distance
        dx = len(msg.ranges)//7
        for j in range(len(msg.ranges)):
            i = j + 0*dx # NOTE: For on the fly tuning
            for k, d in zip(noise_filter_i, noise_filter_dist):
                if (msg.ranges[i] > d) and (msg.ranges[i] < msg.range_max):
                    overflowed = self.pop_fill(msg, i, contender_filter_i, contender_filter_dist)
                    if (sum(noise_filter_dist)/len(noise_filter_dist) < sum(contender_filter_dist)/len(contender_filter_dist)):
                        noise_filter_dist = contender_filter_dist
                        noise_filter_i = contender_filter_i
                    contender_filter_dist = []
                    contender_filter_i = []

        absolute_sensor_position = sum(noise_filter_i) / len(noise_filter_i)
        
        # Reset noise filter
        noise_filter_i = []
        noise_filter_dist = []

        # Get the furthest non negative heading distance
        # NOTE there exists an even more analog way of doing this
        # self._f(v) = ||(self.dir_bias - (heading dot norm(v))) * v||
        _f = lambda v: np.linalg.norm((self.dir_bias - np.dot(old_heading, v / np.linalg.norm(v))) * v)
        i = 0
        self.pop_fill(i, noise_filter_i, noise_filter_dist, f=_f)

        def vad(ang, dist): return np.array([dist*math.cos(ang), dist*math.sin(ang)])
        def adjust(v, delta_ang): 
            rot_mat = [[math.cos(delta_ang), -math.sin(delta_ang)], [-math.sin(delta_ang), math.cos(delta_ang)]]
            return np.matmul(rot_mat, v)
        
        # Check Exploration Graph
        # H0 Done with a temporary actuation graph
        _old_heading = self.old_heading
        drotation = self.live_drot - self.old_rotation
        old_heading = adjust(_old_heading, drotation) # Correct with rot mat ...
        
        # Increments
        da = 2. * math.pi / len(msg.ranges)
        for i in range(len(msg.ranges)):
            for k, d in zip(noise_filter_i, noise_filter_dist):
                if (msg.ranges[i] > d) and (msg.ranges[i] < msg.range_max):
                    overflowed = self.pop_fill(msg, i, contender_filter_i, contender_filter_dist, f=_f)
                    if (sum(noise_filter_dist)/len(noise_filter_dist) < sum(contender_filter_dist)/len(contender_filter_dist)):
                        noise_filter_dist = _f(vad(i*da, contender_filter_dist))
                        noise_filter_i = contender_filter_i
                    contender_filter_dist = []
                    contender_filter_i = []

        exploration_sensor_position = sum(noise_filter_i) / len(noise_filter_i)
        
        if math.abs(absolute_sensor_position - exploration_sensor_position) > 10:
            # Start exploration countdown (heading reset countdown)
            self.timer = self.timer_max
        
        # Actuation
        cutoff = len(msg.ranges)//3
        if sensor_position - cutoff < 0:        self.publ = True
        elif sensor_position - 2*cutoff > 0:    self.pubr = True
        
        self.act_dist = sum(noise_filter_dist)/len(noise_filter_dist)

    def obstacle_avoidance(self,msg):
        self.get_logger().info('Turning')
        dx = len(msg.ranges) // 8

        lwall = False
        rwall = False

        # Left
        for j in range(int(dx*1.1)): # NOTE: this sensor is slightly larger!
            i = j + 2*dx
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                lwall = True
                break

        # Right
        for j in range(dx):
            i = j + 5*dx
            if (msg.ranges[i] < self.act_dist) and (msg.ranges[i] > msg.range_min):
                rwall = True
                break
        
        # Engage motor actions 
        if self.stop == 1: # Stopped
            if rwall: self.lpub = True
            elif lwall: self.rpub = True

    def navigate(self, msg):
        x = len(msg.ranges)//8
        for j in range(x):
            i = j + 3*x
            if (msg.ranges[i] > self.act_dist * 0.9) and (msg.ranges[i] < msg.range_max):
                self.stop = 1
                self.obstacle_avoidance(msg)
            else:
                self.publ = False
                self.pubr = False
                self.stop = 0
                self.navigate(msg)
      
    # NOTE not much better than just optimizing the arc-length
    def exploration_timer(self):
        """
        Only a certain amount of time is allotted to moving away
        from heading
        """
        self.timer -= 1
    
    def movement_callback(self):
        msg = Twist()
        if (self.stop == 0):
           msg.linear.x = .1
           self.get_logger().info('Moving')

           # velocity 0.1 for 1 second to lidar coords = 0.098 est 
           # TODO Calibrate
           # TODO Check idx correctness should be y but doublecheck
           self.heading[1] += 0.098 * self.clock_speed
        else:
           msg.linear.x = .0
           self.get_logger().info('Stopping')

        # set linear velocity
        self.velocity_.publish(msg)

        # TODO test and calibrate
        if self.lpub:
            self.turnL()
            self.lpub = False
            self.get_logger().info('Turning L')
        elif self.rpub:
            self.turnR()
            self.rpub = False
            self.get_logger().info('Turning R')
        
        # To start exploration timer is set to timer_max
        if self.timer > 0:
            self.exploration_timer()
        elif self.timer == 0:
            self.timer -= 1 # Only reset once (Assumed by now you've nearly found the next direction)
            # Reset heading to new heading
            self.live_dheading = np.array([0, 0])
            self.old_rotation = self.live_drot
            self.old_heading = vad(self.old_rotation, self.live_dheading) # NOTE: Storing the old rotation

    def turnL(self):
        msg = Twist()
        msg.angular.z = -.3 # Good, speed of rotate away from opposing obstacle
        self.velocity_.publish(msg)

        # Called at clock interval (TODO CALIBRATE)
        self.live_drot += -math.pi/12 * self.clock_speed

    def turnR(self):
        msg = Twist()
        msg.angular.z = .3 # Good, speed of rotate away from opposing obstacle
        self.velocity_.publish(msg)

        # Called at clock interval (TODO CALIBRATE)
        self.live_drot += math.pi/12 * self.clock_speed


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
