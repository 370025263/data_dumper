import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # 导入 NavSatFix 消息类型
import mysql.connector
from datetime import datetime
from typing import Callable, Dict, Any

class Config:
    """
    Centralized configuration for default values and database settings.
    """
    # Database configuration
    DB_CONFIG = {
        'host': 'localhost',
        'user': 'test_user',
        'password': 'password123',  # 替换为实际的 MySQL 密码
        'database': 'test'  # 替换为实际的数据库名称
    }

    # Default values
    DEFAULTS = {
        'detection_id': 'DET-001',
        'vehicle_vin': 'GNSS-VIN-001',
        'vehicle_plate': 'UNKNOWN',
        'longitude_default': 0.0,
        'latitude_default': 0.0,
        'user_domain': 'default_domain',
        'user_query_domain': 'default_query_domain',
        'status': 'active',
        'terminal_code': 'UNKNOWN_TERMINAL',
        'vehicle_type': 'UNKNOWN_TYPE',
        'flag': 'N',
        'creater': 'system',
        'modifer': 'system'
    }

class DataManager:
    """
    Manages the latest messages received from various topics.
    """
    def __init__(self):
        self.latest_messages: Dict[str, Any] = {}

    def update_message(self, topic: str, msg: Any):
        """
        Updates the latest message for a given topic.
        """
        self.latest_messages[topic] = msg

    def get_latest_message(self, topic: str):
        """
        Retrieves the latest message for a given topic.
        """
        return self.latest_messages.get(topic, None)

class Listener(Node):
    """
    ROS 2 Node that listens to various data topics and inserts data into MySQL.
    Designed for scalability and reusability.
    """
    def __init__(self, interval=5):
        super().__init__('data_listener')  # 初始化节点，节点名称为 'data_listener'
        self.data_manager = DataManager()
        self.config = Config()
        self.insert_functions: Dict[str, Callable[[datetime], None]] = {}
        self.setup_subscribers()
        self.setup_insert_functions()
        self.timer = self.create_timer(interval, self.timer_callback)  # 使用 ROS 定时器

    def setup_subscribers(self):
        """
        Sets up subscribers for various ROS topics.
        """
        # GNSS Subscriber
        self.gnss_subscription = self.create_subscription(
            NavSatFix,
            '/sensing/gnss/monitor',
            self.gnss_callback,
            10)
        self.gnss_subscription  # 防止未使用变量的警告

        # TODO: 添加更多的订阅者，例如：
        # self.other_subscription = self.create_subscription(
        #     OtherMsgType,
        #     '/sensing/other/topic',
        #     self.other_callback,
        #     10)

    def gnss_callback(self, msg: NavSatFix):
        """
        Callback function for GNSS data. Stores the latest message.
        """
        self.data_manager.update_message('gnss', msg)
        self.get_logger().debug('Received GNSS data.')

    # TODO: 添加更多的回调函数
    # def other_callback(self, msg: OtherMsgType):
    #     self.data_manager.update_message('other', msg)

    def setup_insert_functions(self):
        """
        Registers database insert functions.
        """
        self.insert_functions['gnss'] = self.insert_gnss_data_to_mysql
        # TODO: 添加更多的插入函数，例如：
        # self.insert_functions['other'] = self.insert_other_data_to_mysql

    def timer_callback(self):
        """
        Timer callback that triggers all registered insert functions with a synchronized timestamp.
        """
        current_time = datetime.now()
        for name, insert_func in self.insert_functions.items():
            insert_func(current_time)

    def insert_gnss_data_to_mysql(self, current_time: datetime):
        """
        Inserts the latest GNSS data into the MySQL database using a synchronized timestamp.
        """
        msg: NavSatFix = self.data_manager.get_latest_message('gnss')
        if msg is None:
            self.get_logger().warning('No GNSS data available to insert.')
            return

        try:
            # 建立数据库连接
            connection = mysql.connector.connect(**self.config.DB_CONFIG)
            cursor = connection.cursor()

            # 插入数据的 SQL 语句，包含所有字段
            sql = (
                "INSERT INTO `车辆定位信息表` ("
                "guid, detection_id, vehicle_vin, vehicle_plate, longitude, latitude, "
                "user_domain, user_query_domain, status, terminal_code, vehicle_type, flag, "
                "create_time, modify_time, creater, modifer"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
            )

            data = (
                current_time.strftime('%Y%m%d%H%M%S%f'),  # GUID（使用当前时间生成唯一值）
                self.config.DEFAULTS['detection_id'],     # detection_id
                self.config.DEFAULTS['vehicle_vin'],      # vehicle_vin
                self.config.DEFAULTS['vehicle_plate'],    # vehicle_plate
                msg.longitude if msg.longitude is not None else self.config.DEFAULTS['longitude_default'],  # 经度
                msg.latitude if msg.latitude is not None else self.config.DEFAULTS['latitude_default'],    # 纬度
                self.config.DEFAULTS['user_domain'],      # user_domain
                self.config.DEFAULTS['user_query_domain'],# user_query_domain
                self.config.DEFAULTS['status'],           # status
                self.config.DEFAULTS['terminal_code'],    # terminal_code
                self.config.DEFAULTS['vehicle_type'],     # vehicle_type
                self.config.DEFAULTS['flag'],             # flag
                current_time.strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                current_time.strftime('%Y-%m-%d %H:%M:%S.%f'),  # modify_time
                self.config.DEFAULTS['creater'],          # creater
                self.config.DEFAULTS['modifer']           # modifer
            )

            # 执行插入操作
            cursor.execute(sql, data)
            connection.commit()  # 提交事务
            self.get_logger().info('GNSS data inserted into MySQL.')

        except mysql.connector.Error as err:
            # 捕获 MySQL 错误并打印日志
            self.get_logger().error(f"Failed to write GNSS data to MySQL: {err}")
            # 根据需求决定是否退出程序，这里选择不退出
            # raise SystemExit(1)

        finally:
            # 确保关闭数据库连接和游标
            if connection.is_connected():
                cursor.close()
                connection.close()

    # TODO: 可以在这里添加更多的插入函数
    # def insert_other_data_to_mysql(self, current_time: datetime):
    #     msg = self.data_manager.get_latest_message('other')
    #     if msg is None:
    #         self.get_logger().warning('No other data available to insert.')
    #         return
    #     try:
    #         connection = mysql.connector.connect(**self.config.DB_CONFIG)
    #         cursor = connection.cursor()
    #         sql = (
    #             "INSERT INTO `其他表` ("
    #             "guid, other_field, create_time, modify_time, creater, modifer"
    #             ") VALUES (%s, %s, %s, %s, %s, %s)"
    #         )
    #         data = (
    #             current_time.strftime('%Y%m%d%H%M%S%f'),  # GUID
    #             msg.some_field,                          # other_field
    #             current_time.strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
    #             current_time.strftime('%Y-%m-%d %H:%M:%S.%f'),  # modify_time
    #             self.config.DEFAULTS['creater'],          # creater
    #             self.config.DEFAULTS['modifer']           # modifer
    #         )
    #         cursor.execute(sql, data)
    #         connection.commit()
    #         self.get_logger().info('Other data inserted into MySQL.')
    #     except mysql.connector.Error as err:
    #         self.get_logger().error(f"Failed to write other data to MySQL: {err}")
    #     finally:
    #         if connection.is_connected():
    #             cursor.close()
    #             connection.close()

    def destroy_node(self):
        """
        Cleans up before shutting down the node.
        """
        super().destroy_node()

def main(args=None):
    """
    The main entry point for the ROS 2 node.
    """
    rclpy.init(args=args)  # 初始化 ROS 2 Python 客户端库
    listener = Listener(interval=5)  # 创建节点实例，默认每 5 秒写入一次
    try:
        rclpy.spin(listener)  # 保持节点运行，等待消息
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()  # 销毁节点
        rclpy.shutdown()  # 关闭 ROS 2 Python 客户端库

if __name__ == '__main__':
    main()

