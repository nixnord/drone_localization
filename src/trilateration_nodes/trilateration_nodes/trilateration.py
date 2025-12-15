import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy

class TrilaterationNode(Node):

    def __init__(self):
        super().__init__("trilateration_node")
        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/lora/range",
            self.calculate_position_callback,
            10
        )
        self.pub =  self.create_publisher(
            Float64MultiArray,
            "/drone/estimatedpose",
            10
        )

        self.gs_vectors = numpy.array([
            [0, 0], # GS1
            [80, 0], # GS2
            [40, 69.28] # GS3
        ])

        self.get_logger().info("Trilateration node has started!")

    def calculate_position_callback(self, array: Float64MultiArray):
        '''
        (x - x1)**2 + (y - y1)**2 = d1**2
        (x - x2)**2 + (y - y2)**2 = d2**2
        (x - x3)**2 + (y - y3)**2 = d3**2
        Solve using linear algebra
        Ax = B
        A: numpy array (2x2 matrix)
        B: numpy array (2x1 matrix)
        '''
        dist = array.data # [ d1, d2, d3, height ]
        x1, y1 = self.gs_vectors[0]
        d1 = dist[0]
        A = []
        B = []
        for vector, d in zip(self.gs_vectors[1:], dist[1:3]):
            arr1 = [ 2*(vector[0] - x1), 2*(vector[1] - y1) ]
            constant = d1**2 - d**2 - x1**2 + vector[0]**2 - y1**2 + vector[1]**2
            A.append(arr1)
            B.append(constant)
        A = numpy.array(A); B = numpy.array(B)
        try:
            pos, r, ra, s = numpy.linalg.lstsq(A, B, rcond=None)
            message = Float64MultiArray()
            message.data = [float(pos[0]), float(pos[1]), float(dist[3])] # [ x, y, z ]
            self.pub.publish(message)
        except Exception as e:
            self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)
    node = TrilaterationNode()
    rclpy.spin(node)
    rclpy.shutdown()
