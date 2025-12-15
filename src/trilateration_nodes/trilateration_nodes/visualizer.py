import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseArray, PoseStamped
from time import sleep
from nav_msgs.msg import Path

class Visualizer(Node):

    def __init__(self):
        super().__init__("visualizernode")
        
        # subscription to get estimated position
        self.sub1 = self.create_subscription(
            Float64MultiArray,
            "/drone/estimatedpose",
            self.estimated_callback,
            10
        )

        # subscription to get real position
        self.sub2 = self.create_subscription(
            PoseArray,
            "/world/trilateration_world/dynamic_pose/info",
            self.truth_callback,
            10
        )

        self.true_position = None

        # publish real time estimated position of the drone
        self.pub1 = self.create_publisher(
            PoseStamped,
            "/drone/estpose",
            10
        )

        # publish real time true position of the drone
        self.pub2 = self.create_publisher(
            PoseStamped,
            "/drone/realpose",
            10
        )

        # publish the real time estimated path of the drone
        self.pub3 = self.create_publisher(
            Path,
            "/drone/estpath",
            10
        )

        # publish the real time true path of the drone
        self.pub4 = self.create_publisher(
            Path,
            "/drone/realpath",
            10
        )

        self.estimated_path = Path()
        self.real_path = Path()

        self.estimated_path.header.frame_id = "world"
        self.real_path.header.frame_id = "world"

        self.get_logger().info("Feedback node has started!")

    def truth_callback(self, posearray: PoseArray):
        #self.get_logger().info("received true position!!")
        self.true_position = posearray

    def estimated_callback(self, array: Float64MultiArray):

        if self.true_position is None or len(self.true_position.poses) == 0: return

        estpos = PoseStamped()
        realpos = PoseStamped()

        pose = self.true_position.poses[0]
        drone_true_position = [pose.position.x, pose.position.y, pose.position.z]

        estpos.header.stamp = self.get_clock().now().to_msg()
        estpos.header.frame_id = "world"
        estpos.pose.position.x = array.data[0]
        estpos.pose.position.y = array.data[1]
        estpos.pose.position.z = array.data[2]
        estpos.pose.orientation.w = 1.0
        estpos.pose.orientation.x = 0.0
        estpos.pose.orientation.y = 0.0
        estpos.pose.orientation.z = 0.0

        self.estimated_path.poses.append(estpos)
        self.estimated_path.header.stamp = estpos.header.stamp


        realpos.header.stamp = self.get_clock().now().to_msg()
        realpos.header.frame_id = "world"
        realpos.pose.position.x = drone_true_position[0]
        realpos.pose.position.y = drone_true_position[1]
        realpos.pose.position.z = drone_true_position[2]
        realpos.pose.orientation.w = 1.0
        realpos.pose.orientation.x = 0.0
        realpos.pose.orientation.y = 0.0
        realpos.pose.orientation.z = 0.0

        self.real_path.poses.append(realpos)
        self.real_path.header.stamp = realpos.header.stamp

        #if len(self.estimated_path.poses) > 50:
            #self.estimated_path.poses.pop(0)

        self.pub1.publish(estpos)
        self.pub2.publish(realpos)
        self.pub3.publish(self.estimated_path)
        self.pub4.publish(self.real_path)

        self.get_logger().info(f"TRUE POSE: {drone_true_position} | EST POSE: {list(array.data)}")
        sleep(0.3)

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    rclpy.spin(node)
    rclpy.shutdown()
