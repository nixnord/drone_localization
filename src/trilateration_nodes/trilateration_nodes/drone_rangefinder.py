import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float64MultiArray
import numpy

# calculate the distance of the drone from each ground
# station and then add some noise in order to simulate the
# noise when we are actually using a LoRa module

class RangeFinder(Node):

    def __init__(self):
        super().__init__("drone_ranging")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self._subscription = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._position_callback,
            qos_profile
        )
        self._publisher = self.create_publisher(
            Float64MultiArray, 
            "/lora/range", 
            10
        )
        # set a realistic rate of 5Hz
        self.create_timer(0.2, self._publish_ranges)
        # hardcoded positions of the ground stations
        self._gs_vectors = numpy.array([
            [ 0.0,   34.64],   # GS1 — North vertex
            [-30.0, -17.32],   # GS2 — South-West vertex
            [ 30.0, -17.32],   # GS3 — South-East vertex
        ])
        self._latest_ranges = None

        self._standard_deviation = 5.0 # SD of error in distance calculation (near ideal)
        self.get_logger().info("RangeFinder node has started!")

    def _position_callback(self, position: VehicleLocalPosition):
        x_enu, y_enu = position.y, position.x
        drone_vector = numpy.array([x_enu, y_enu])

        distances = []
        # calculate the distance of each ground station from the drone
        for ground_station in self._gs_vectors:
            distance = numpy.linalg.norm(drone_vector - ground_station)
            error = numpy.random.normal(0.0, self._standard_deviation)
            noisy_distance = max(float(distance) + error, 0.0)
            distances.append(float(noisy_distance))
        # cache the latest data
        self._latest_ranges = distances

    def _publish_ranges(self):
        if self._latest_ranges is None:
            return
        message = Float64MultiArray()
        message.data = self._latest_ranges
        self._publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node = RangeFinder()
    rclpy.spin(node)
    rclpy.shutdown()