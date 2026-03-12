import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleAttitude, SensorCombined, VehicleLocalPosition
from scipy.spatial.transform import Rotation
import time
from nav_msgs.msg import Path
from trilateration_nodes.kalman_filter import KalmanFilterConstantAcceleration
import numpy

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

GRAVITY_ENU  = numpy.array([0.0, 0.0, -9.81])
R_NED_TO_ENU = Rotation.from_euler('ZX', [90, 180], degrees=True)
R_LORA = numpy.diag([4.0, 4.0])
R_IMU = numpy.diag([0.01, 0.01])

def convert_ned_to_enu(x, y, z):
    return float(y), float(x), float(-z)

class Visualizer(Node):

    def __init__(self):
        super().__init__("visualizernode")

        self._kf: KalmanFilterConstantAcceleration = None
        self._attitude_q: numpy.ndarray = None
        self._last_timestamp: float = None # last time stamp in microseconds for calculating dt
        self._ax_enu: float = None
        self._ay_enu: float = None
        self._x_lora_enu: float = None
        self._y_lora_enu: float = None
        self._true_position: list = None
        
        # this is to get lora readings
        self._sub_estimated_pose = self.create_subscription(
            Float64MultiArray,
            "/drone/estimatedpose",
            self._estimated_callback,
            10
        )

        # this is to get the true position readings
        self._sub_true_pose = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._truth_callback,
            QOS_PROFILE
        )

        # this is to get the IMU readings
        self._sub_imu = self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self._imu_callback,
            QOS_PROFILE
        )

        self._sub_attitude = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._attitude_callback,
            QOS_PROFILE
        )

        # publish real time estimated position of the drone
        self._pub_estimated_pose = self.create_publisher( PoseStamped, "/drone/estpose", 10)
        # publish real time true position of the drone
        self._pub_true_pose = self.create_publisher( PoseStamped, "/drone/truepose", 10 )
        # publish the real time estimated path of the drone
        self._pub_estimated_path = self.create_publisher( Path, "/drone/estpath", 10 )
        # publish the real time true path of the drone
        self._pub_true_path = self.create_publisher( Path, "/drone/truepath", 10 )
        # publish the fitered position of the drone
        self._pub_filtered_pose = self.create_publisher( PoseStamped, "/drone/filteredpose", 10)
        # publish the filtered path of the drone
        self._pub_filtered_path = self.create_publisher( Path, "/drone/filteredpath", 10)

        self._estimated_path = self._init_path()
        self._real_path = self._init_path()
        self._filtered_path = self._init_path() 

    def _init_path(self) -> Path:
        p = Path()
        p.header.frame_id = "world"
        return p
    
    def _make_pose(self, x: float, y: float, z: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "world"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        return ps
    
    def _rotate_accel_to_enu(self, ax: float, ay: float, az: float) -> numpy.ndarray:
        """
        Rotate body FRD accel → ENU and remove gravity in one step.
        thnx claude
        """
        q = self._attitude_q
        r_body_to_ned = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        r_body_to_enu = R_NED_TO_ENU * r_body_to_ned
        return r_body_to_enu.apply(numpy.array([ax, ay, az])) - GRAVITY_ENU
    
    def _compute_dt(self) -> float:
        now = time.perf_counter()
        if not self._last_timestamp:
            self._last_timestamp = now
            return None
        dt = (now - self._last_timestamp)
        self._last_timestamp = now
        return dt


    def _truth_callback(self, vehiclePosition: VehicleLocalPosition) -> None:
        x_enu, y_enu, z_enu = convert_ned_to_enu(
            vehiclePosition.x,
            vehiclePosition.y,
            vehiclePosition.z
        )
        self._true_position = [x_enu, y_enu, z_enu]

    def _attitude_callback(self, attitude: VehicleAttitude) -> None:
        self._attitude_q = numpy.array(list(attitude.q)[:4])

    def _imu_callback(self, sensorData: SensorCombined) -> None:
        if self._attitude_q is None: return # skip if there is no quaternion value
        acclereration_enu = self._rotate_accel_to_enu(
            sensorData.accelerometer_m_s2[0],
            sensorData.accelerometer_m_s2[1],
            sensorData.accelerometer_m_s2[2]
        )
        self._ax_enu = float(acclereration_enu[0])
        self._ay_enu = float(acclereration_enu[1])
        # keep on updating acceleration with the latest values
        # but if there is no kf instance, skip the rest steps
        if self._kf is None: return
        # the below codeblock will be executed only when there is a kf instance
        # that means acceleration will have its latest values but we need the first
        # lora callback before this gets executed
        dt = self._compute_dt()
        self._kf.predict_state(dt, sigma_x=0.01, sigma_y=0.01)
        self._kf.update_state_acceleration(
            [self._ax_enu, self._ay_enu],
            R_IMU
        )
        # self._publish_values()

    def estimated_callback(self, array: Float64MultiArray):

        if self._true_position is None: return

        drone_est_position = array.data
        if not array.data: return

        dt = self._compute_dt()
        if dt is None:
            # for the first call, dt is initially None. so the initial positions
            # and the current time will be cached so that in the next call
            # dt and the displacement can be calculated for vx and vy
            self._x_lora_enu = float(drone_est_position[0])
            self._y_lora_enu = float(drone_est_position[1])
            return

        if self._kf is None:
            # for exising dt, calculate the displacments
            dx = float(drone_est_position[0]) - self._x_lora_enu
            dy = float(drone_est_position[1]) - self._y_lora_enu
            self._kf = KalmanFilterConstantAcceleration(
                drone_est_position[0], drone_est_position[1],
                dx/dt, dy/dt, self._ax_enu, self._ay_enu,
                0.2, 0.2, 0.04, 0.041, 0.01, 0.012
            )
            return
        
        self._x_lora_enu = float(drone_est_position[0])
        self._y_lora_enu = float(drone_est_position[1])

        self._kf.predict_state(dt, 0.065, 0.066)
        measurement = [self._x_lora_enu, self._y_lora_enu]
        self._kf.update_state_position(measurement, R_LORA)

        drone_true_position = self._true_position[:2]

        kalman_x = self.kf.state_vector[0,0]
        kalman_y = self.kf.state_vector[1,0]

        estpos = self._make_pose(self._x_lora_enu, self._y_lora_enu, 0.0)
        self._estimated_path.poses.append(estpos)
        self._estimated_path.header.stamp = estpos.header.stamp
        realpos = self._make_pose(float(drone_true_position[0]), float(drone_true_position[1]), 0.0)
        self._real_path.poses.append(estpos)
        self._real_path.header.stamp = realpos.header.stamp
        filtered_pose = self._make_pose(kalman_x, kalman_y, 0.0)
        self._filtered_path.poses.append(filtered_pose)
        self._filtered_path.header.stamp = filtered_pose.header.stamp
        #if len(self.estimated_path.poses) > 50:
            #self.estimated_path.poses.pop(0)

        self._pub_estimated_pose.publish(estpos)
        self._pub_true_pose.publish(realpos)
        self._pub_estimated_path.publish(self._estimated_path)
        self._pub_true_path.publish(self._real_path)
        self._pub_filtered_pose.publish(filtered_pose)
        self._pub_filtered_path.publish(self._filtered_path)

        self.get_logger.info(f"""

TP:{drone_true_position[0]:.2f},{drone_true_position[1]:.2f}
EP:{self._x_lora_enu:.2f},{self._y_lora_enu:.2f}
KF:{kalman_x:.2f},{kalman_y:.2f}

""")

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    rclpy.spin(node)
    rclpy.shutdown()
