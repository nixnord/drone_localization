import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
import numpy


class RangeFinder(Node):

    def __init__(self):
        super().__init__("rangefinder")
        self.subscription = self.create_subscription(
            PoseArray,
            "/world/trilateration_world/dynamic_pose/info",
            self.calculate_distance_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            "/lora/range", 
            10
        )

        self.gs_vectors = numpy.array([
            [0, 0],
            [80, 0],
            [40, 69.28]
        ])

        self.standard_deviation = 5.0 # SD of error in distance calculation (near ideal)

        self.get_logger().info("RangeFinder node has started!")

    def calculate_distance_callback(self, array: PoseArray):
        '''
        Calculate the Euclidean distance between drone vector
        and each of the ground station vectors.
        Also adds gaussian stochastic error in order to add reality
        to simulating LoRa communications
        
        :param self: node instance
        :param array: the message type to receive
        :type array: PoseArray
        '''

        if len(array.poses) == 0:
            return
        position = array.poses[0]
        drone_vector = numpy.array([
            position.position.x,
            position.position.y
        ])
        height = position.position.z

        distances = []

        for ground_station in self.gs_vectors:
            distance = numpy.linalg.norm(drone_vector - ground_station)
            error = numpy.random.normal(0.0, self.standard_deviation) # adding stochastic errors
            noisy_distance = max(float(distance) + error, 0.0) # distance cannot be negative
            distances.append(noisy_distance)
        
        distances.append(height) # [ d1, d2, d3, height ]

        message = Float64MultiArray()
        message.data = distances
        self.publisher.publish(message)

        #self.get_logger().info(f"Message Published: {message}") for debug

def main(args=None):
    rclpy.init(args=args)
    node = RangeFinder()
    rclpy.spin(node)
    rclpy.shutdown()