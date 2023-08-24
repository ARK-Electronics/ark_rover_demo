import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
import subprocess
import os
import signal

class LidarCheckNode(Node):
    def __init__(self):
        super().__init__('lidar_check_node')
        self.timer = time.time()
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        self.timer = time.time()  # Reset the timer whenever a scan message is received

def check_lidar_node(node):
    print(node.timer)
    return time.time() - node.timer < 10  # Check if the timer has grown to 5 seconds

def kill_lidar_process():
    pids = subprocess.getoutput("pgrep -f rplidar_composition")
    if pids:
        for pid in pids.splitlines():
            try:
                os.kill(int(pid), signal.SIGKILL)
            except ProcessLookupError:
                print(f"Process with PID {pid} not found, continuing...")
    else:
        print("No matching processes found")


def run_lidar_process():
    command = ["ros2", "run", "rplidar_ros", "rplidar_composition", "--ros-args", "-p", "serial_port:=/dev/ttyUSB0", "-p", "frame_id:=laser_frame", "-p", "angle_compensate:=true", "-p", "scan_mode:=Standard"]
    subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    rclpy.init()
    lidar_check_node = LidarCheckNode()

    # Start the lidar process initially
    run_lidar_process()

    while rclpy.ok():
        print("checking")
        rclpy.spin_once(lidar_check_node, timeout_sec=0.1)

        if not check_lidar_node(lidar_check_node):
            print("RPLidar node appears unresponsive. Restarting...")
            kill_lidar_process() # Kill the existing process
            run_lidar_process()  # Restart the node

        time.sleep(8) # Check continuously

    rclpy.shutdown()

if __name__ == '__main__':
    main()
