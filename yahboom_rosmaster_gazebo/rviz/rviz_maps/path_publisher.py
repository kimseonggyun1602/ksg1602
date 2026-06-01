import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, '/planned_path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # 1초마다 경로 퍼블리시
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.load_waypoints()

    def load_waypoints(self):
        pkg_share = get_package_share_directory('yahboom_rosmaster_gazebo')
        file_path = os.path.join(pkg_share, 'rviz', 'rviz_maps', 'waypoints.txt')
        
        if not os.path.exists(file_path):
            self.get_logger().error(f"Cannot find {file_path}")
            return
            
        with open(file_path, 'r') as f:
            lines = f.readlines()
            
        for line in lines:
            x_str, y_str = line.strip().split(',')
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x_str)
            pose.pose.position.y = float(y_str)
            pose.pose.position.z = 0.0
            
            # 메카넘 휠 특성상 특정 회전(Orientation)이 필요하다면 여기서 쿼터니언 설정
            pose.pose.orientation.w = 1.0 
            
            self.path_msg.poses.append(pose)
            
        self.get_logger().info("Waypoints successfully loaded into Path message!")

    def timer_callback(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()