import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # 导入 NavSatFix 消息类型

class GNSSListener(Node):
    def __init__(self):
        super().__init__('gnss_listener')  # 初始化节点，节点名称为 'gnss_listener'
        # 创建一个订阅者，订阅 /sensing/gnss/oxts/nav_sat_fix topic，消息类型为 NavSatFix
        self.subscription = self.create_subscription(
            NavSatFix,
            '/sensing/gnss/oxts/nav_sat_fix',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量的警告

    def listener_callback(self, msg):
        # 打印接收到的消息
        self.get_logger().info(f'Received GNSS data:')
        self.get_logger().info(f'  Header: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
        self.get_logger().info(f'  Latitude: {msg.latitude}')
        self.get_logger().info(f'  Longitude: {msg.longitude}')
        self.get_logger().info(f'  Altitude: {msg.altitude}')
        self.get_logger().info(f'  Position Covariance: {msg.position_covariance}')
        self.get_logger().info(f'  Position Covariance Type: {msg.position_covariance_type}')

def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS 2 Python 客户端库
    gnss_listener = GNSSListener()  # 创建节点实例
    rclpy.spin(gnss_listener)  # 保持节点运行，等待消息
    gnss_listener.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭 ROS 2 Python 客户端库

if __name__ == '__main__':
    main()
