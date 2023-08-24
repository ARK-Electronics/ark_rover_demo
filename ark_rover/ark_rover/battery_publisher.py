import rclpy
from rclpy.node import Node
from px4_msgs.msg import BatteryStatus
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class BatteryStatusNode(Node):
    def __init__(self):
        super().__init__('battery_status_node')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.listener_callback,
            qos)
        self.publisher = self.create_publisher(Float32, '/battery_remaining', qos)
        self.subscription

    def listener_callback(self, msg):
        remaining_battery = msg.remaining
        self.publish_remaining(remaining_battery)

    def publish_remaining(self, value):
        msg = Float32()
        msg.data = value
        self.publisher.publish(msg)
        # self.get_logger().info(f'Published remaining battery status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
