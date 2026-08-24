from dataclasses import dataclass
import threading
import copy


@dataclass
class VehicleStateData:
    # Timestamp (nanoseconds from FC)
    timestamp: int = 0

    # Position (m)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # Velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Quaternion
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    # Angular velocity (rad/s)
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    # Linear acceleration (m/s²)
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0


class VehicleState:

    def __init__(self):
        self._lock = threading.Lock()
        self._data = VehicleStateData()

    def update_pose(self, msg):
        with self._lock:
            self._data.timestamp = (
                msg.header.stamp.sec * 1_000_000_000
                + msg.header.stamp.nanosec
            )

            self._data.x = msg.pose.position.x
            self._data.y = msg.pose.position.y
            self._data.z = msg.pose.position.z

            self._data.qx = msg.pose.orientation.x
            self._data.qy = msg.pose.orientation.y
            self._data.qz = msg.pose.orientation.z
            self._data.qw = msg.pose.orientation.w

    def update_twist(self, msg):
        with self._lock:
            self._data.vx = msg.twist.linear.x
            self._data.vy = msg.twist.linear.y
            self._data.vz = msg.twist.linear.z

    def update_imu(self, msg):
        with self._lock:
            self._data.p = msg.angular_velocity.x
            self._data.q = msg.angular_velocity.y
            self._data.r = msg.angular_velocity.z

            self._data.ax = msg.linear_acceleration.x
            self._data.ay = msg.linear_acceleration.y
            self._data.az = msg.linear_acceleration.z

    def copy(self):
        with self._lock:
            return copy.deepcopy(self._data)