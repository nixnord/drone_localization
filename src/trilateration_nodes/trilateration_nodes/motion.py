import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

#TODO: write motion node for the PX4 drone

class ObjectMover(Node):
    def __init__(self):
        super().__init__("mobility")
        self.pub = self.create_publisher(
            Twist,
            "/model/drone/cmd_vel",
            10
        )
        self.timer = self.create_timer(0.01, self.callback_func)
        self.direction = 1 # 1 (x) --> 2(y) --> 3(-x) --> 4(-y) (repeat)
        self.distance_covered = 0
        self.linear_velocity = 1.2

    def callback_func(self):
        msg = Twist()
        self.linear_velocity += 0.01
        msg.linear.x = self.linear_velocity
        msg.angular.z = 0.05
        self.pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = ObjectMover()
    rclpy.spin(node)
    rclpy.shutdown()

'''
square

if self.direction == 1:
            if self.distance_covered < 100.0:
                msg.linear.x = 2.0
                self.distance_covered += 2.0 * 0.01
            else:
                self.direction += 1
                self.distance_covered = 0
        elif self.direction == 2:
            if self.distance_covered < 100.0:
                msg.linear.y = 2.0
                self.distance_covered += 2.0 * 0.01
            else:
                self.direction += 1
                self.distance_covered = 0
        elif self.direction == 3:
            if self.distance_covered < 100.0:
                msg.linear.x = -2.0
                self.distance_covered += 2.0 * 0.01
            else:
                self.direction += 1
                self.distance_covered = 0
        else:
            if self.distance_covered < 100.0:
                msg.linear.y = -2.0
                self.distance_covered += 2.0 * 0.01
            else:
                self.direction = 1
                self.distance_covered = 0


msg.linear.x = 1.2
        msg.angular.z = 0.05
'''
