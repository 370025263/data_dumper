"""
README

添加新的表的方法:
1. 添加订阅topic
2. 为topic的消息写一个回调(更新DataManager)
3. 从Config和DataManager中写入数据到Mysql表（如果有新的默认字段记得更新Config)

HINTS:
- Config中保存了一些默认数值，用于多个表之间公用。
- DataManager负责一些需要保证表间同步的字段。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # 导入 NavSatFix 消息类型
from nav_msgs.msg import Odometry  # 导入 Odometry 消息类型
from autoware_planning_msgs.msg import Trajectory  # 导入 Trajectory 消息类型
import mysql.connector
from datetime import datetime
from typing import Callable, Dict, Any
import json
import math

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
        'status': 'valid',
        'terminal_code': 'UNKNOWN_TERMINAL',
        'vehicle_type': 'UNKNOWN_TYPE',
        'flag': 'N',
        'creater': 'system',
        'modifer': 'system',
        # 新增默认值
        'vehicle_id': 'VEHICLE-001',  # 根据实际情况修改
        'start_time_default': None,
        'end_time_default': None,
        'duration_default': None,
        'path_default': None,
        'terminal_sim_default': 'UNKNOWN_SIM',
        'factory_default': 'UNKNOWN_FACTORY',
        'task_code_default': 'TASK-001',
        'driver_default': 'UNKNOWN_DRIVER',
        'contacts_name_default': 'UNKNOWN',
        'vehicle_guid_default': 'UNKNOWN_GUID',
        # 新增表的默认值
        'data_uuid_default': 'DATA_UUID_001',
        'vin_default': 'VIN_001',
        'event_time_default': '2025-01-04T00:00:00',  # 根据实际需求调整
        'automatic_driving_data_recording_system_hardware_version_default': 'v1.0',
        'automatic_driving_data_recording_system_serial_num_default': 'SN001',
        'automatic_driving_data_recording_system_software_version_default': 'SW1.0',
        'event_type_default': 'EVENT_TYPE_001',
        'event_record_complete_flag_default': 'Y',
        'vehicle_heading_angle_default': 0.0,  # 改为数值类型
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
        super().__init__('data_dumper')  # 初始化节点，节点名称为 'data_dumper'
        self.data_manager = DataManager()
        self.config = Config()
        self.insert_functions: Dict[str, Callable[[int], None]] = {}
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

        # Trajectory Subscriber
        self.trajectory_subscription = self.create_subscription(
            Trajectory,  # 修改为 Trajectory 消息类型
            '/planning/scenario_planning/trajectory',
            self.trajectory_callback,
            10)
        self.trajectory_subscription  # 防止未使用变量的警告

        # Odometry Subscriber
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odometry_callback,
            10)
        self.odometry_subscription  # 防止未使用变量的警告

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
        self.get_logger().debug('Received new GNSS data.')

    def trajectory_callback(self, msg: Trajectory):
        """
        Callback function for Trajectory data. Stores the latest message.
        """
        self.data_manager.update_message('trajectory', msg)
        self.get_logger().info('Received new Trajectory data.')

    def odometry_callback(self, msg: Odometry):
        """
        Callback function for Odometry data. Stores the latest message.
        """
        self.data_manager.update_message('odometry', msg)
        self.get_logger().debug('Received Odometry data.')

    # TODO: 添加更多的回调函数
    # def other_callback(self, msg: OtherMsgType):
    #     self.data_manager.update_message('other', msg)

    def setup_insert_functions(self):
        """
        Registers database insert functions.
        """
        self.insert_functions['gnss'] = self.insert_gnss_data_to_mysql
        self.insert_functions['trajectory'] = self.insert_trajectory_data_to_mysql
        self.insert_functions['odometry'] = self.insert_odometry_to_mysql
        # TODO: 添加更多的插入函数，例如：
        # self.insert_functions['other'] = self.insert_other_data_to_mysql

    def timer_callback(self):
        """
        Timer callback that triggers all registered insert functions with a synchronized timestamp.
        """
        current_id = int(datetime.now().timestamp() * 1000)  # 使用当前时间戳（毫秒）
        for name, insert_func in self.insert_functions.items():
            insert_func(current_id)

    def insert_gnss_data_to_mysql(self, current_id: int):
        """
        Inserts the latest GNSS data into the MySQL database using a synchronized timestamp as ID.
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
                current_id,  # guid（当前时间戳）
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
                datetime.fromtimestamp(current_id / 1000.0).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                datetime.fromtimestamp(current_id / 1000.0).strftime('%Y-%m-%d %H:%M:%S.%f'),  # modify_time
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
            # 可以根据需求选择是否采取进一步措施，例如重试或报警

        finally:
            # 确保关闭数据库连接和游标
            if connection.is_connected():
                cursor.close()
                connection.close()

    def insert_trajectory_data_to_mysql(self, current_id: int):
        """
        Inserts the latest Trajectory data into the MySQL database using a synchronized timestamp as ID.
        """
        msg: Trajectory = self.data_manager.get_latest_message('trajectory')
        if msg is None:
            self.get_logger().warning('No Trajectory data available to insert.')
            return

        try:
            # 将 Trajectory 数据序列化为 JSON
            trajectory_json = json.dumps(self.serialize_trajectory(msg), ensure_ascii=False)

            # 建立数据库连接
            connection = mysql.connector.connect(**self.config.DB_CONFIG)
            cursor = connection.cursor()

            # 插入数据的 SQL 语句，包含所有字段
            sql = (
                "INSERT INTO `车辆定位信息轨迹表` ("
                "guid, vehicle_id, vehicle_plate, creater, create_time, modify_time, modifer, flag, "
                "user_domain, user_query_domain, status, start_time, end_time, duration, path, "
                "vehicle_vin, terminal_sim, factory, mileage, task_code, driver, contacts_name, vehicle_guid"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
            )

            data = (
                current_id,  # guid（当前时间戳）
                self.config.DEFAULTS['vehicle_id'],        # vehicle_id
                self.config.DEFAULTS['vehicle_plate'],     # vehicle_plate
                self.config.DEFAULTS['creater'],          # creater
                datetime.fromtimestamp(current_id / 1000.0).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                datetime.fromtimestamp(current_id / 1000.0).strftime('%Y-%m-%d %H:%M:%S.%f'),  # modify_time
                self.config.DEFAULTS['modifer'],           # modifer
                self.config.DEFAULTS['flag'],              # flag
                self.config.DEFAULTS['user_domain'],       # user_domain
                self.config.DEFAULTS['user_query_domain'], # user_query_domain
                self.config.DEFAULTS['status'],            # status
                self.config.DEFAULTS['start_time_default'],# start_time
                self.config.DEFAULTS['end_time_default'],  # end_time
                self.config.DEFAULTS['duration_default'],  # duration
                trajectory_json,                            # path
                self.config.DEFAULTS['vehicle_vin'],       # vehicle_vin
                self.config.DEFAULTS['terminal_sim_default'],# terminal_sim
                self.config.DEFAULTS['factory_default'],    # factory
                None,                                       # mileage (填NULL)
                self.config.DEFAULTS['task_code_default'], # task_code
                self.config.DEFAULTS['driver_default'],    # driver
                self.config.DEFAULTS['contacts_name_default'], # contacts_name
                self.config.DEFAULTS['vehicle_guid_default']   # vehicle_guid
            )

            # 执行插入操作
            cursor.execute(sql, data)
            connection.commit()  # 提交事务
            self.get_logger().info('Trajectory data inserted into MySQL.')

        except mysql.connector.Error as err:
            # 捕获 MySQL 错误并打印日志
            self.get_logger().error(f"Failed to write Trajectory data to MySQL: {err}")
            # 可以根据需求选择是否采取进一步措施，例如重试或报警

        finally:
            # 确保关闭数据库连接和游标
            if connection.is_connected():
                cursor.close()
                connection.close()

    def insert_odometry_to_mysql(self, current_id: int):
        """
        Inserts the latest Odometry data into the MySQL database using a synchronized timestamp as ID.
        """
        msg: Odometry = self.data_manager.get_latest_message('odometry')
        if msg is None:
            self.get_logger().warning('No Odometry data available to insert.')
            return

        try:
            # 计算 yaw（航向角）从四元数
            orientation = msg.pose.pose.orientation
            yaw = self.quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

            # 建立数据库连接
            connection = mysql.connector.connect(**self.config.DB_CONFIG)
            cursor = connection.cursor()

            # 插入数据的 SQL 语句，包含所有字段
            sql = (
                "INSERT INTO `车辆及自动驾驶数据记录系统基本信息表` ("
                "id, data_uuid, vin, event_time, "
                "automatic_driving_data_recording_system_hardware_version, "
                "automatic_driving_data_recording_system_serial_num, "
                "automatic_driving_data_recording_system_software_version, "
                "event_type, event_record_complete_flag, accumulated_driving_distance, "
                "vehicle_heading_angle, longitude, latitude, create_time, modify_time"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
            )

            data = (
                current_id,  # id（当前时间戳）
                self.config.DEFAULTS['data_uuid_default'],  # data_uuid
                self.config.DEFAULTS['vin_default'],         # vin
                self.config.DEFAULTS['event_time_default'],  # event_time
                self.config.DEFAULTS['automatic_driving_data_recording_system_hardware_version_default'],  # hardware_version
                self.config.DEFAULTS['automatic_driving_data_recording_system_serial_num_default'],  # serial_num
                self.config.DEFAULTS['automatic_driving_data_recording_system_software_version_default'],  # software_version
                self.config.DEFAULTS['event_type_default'],   # event_type
                self.config.DEFAULTS['event_record_complete_flag_default'],  # event_record_complete_flag
                None,  # accumulated_driving_distance (填NULL)
                yaw,   # vehicle_heading_angle
                msg.pose.pose.position.x if msg.pose.pose.position.x is not None else self.config.DEFAULTS['longitude_default'],  # longitude
                msg.pose.pose.position.y if msg.pose.pose.position.y is not None else self.config.DEFAULTS['latitude_default'],    # latitude
                datetime.fromtimestamp(current_id / 1000.0).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                datetime.fromtimestamp(current_id / 1000.0).strftime('%Y-%m-%d %H:%M:%S.%f')   # modify_time
            )

            # 执行插入操作
            cursor.execute(sql, data)
            connection.commit()  # 提交事务
            self.get_logger().info('Odometry data inserted into MySQL.')

        except mysql.connector.Error as err:
            # 捕获 MySQL 错误并打印日志
            self.get_logger().error(f"Failed to write Odometry data to MySQL: {err}")
            # 可以根据需求选择是否采取进一步措施，例如重试或报警

        finally:
            # 确保关闭数据库连接和游标
            if connection.is_connected():
                cursor.close()
                connection.close()

    def quaternion_to_yaw(self, x, y, z, w) -> float:
        """
        Converts a quaternion into a yaw angle (in radians).

        :param x: 四元数的 x 分量
        :param y: 四元数的 y 分量
        :param z: 四元数的 z 分量
        :param w: 四元数的 w 分量
        :return: 航向角 (Yaw)，单位为弧度
        """
        # 计算航向角公式参考: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def serialize_trajectory(self, msg: Trajectory) -> Dict[str, Any]:
        """
        Serializes the Trajectory message into a dictionary suitable for JSON storage.
        """
        return {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'points': [
                {
                    'time_from_start': {
                        'sec': point.time_from_start.sec,
                        'nanosec': point.time_from_start.nanosec
                    },
                    'pose': {
                        'position': {
                            'x': point.pose.position.x,
                            'y': point.pose.position.y,
                            'z': point.pose.position.z
                        },
                        'orientation': {
                            'x': point.pose.orientation.x,
                            'y': point.pose.orientation.y,
                            'z': point.pose.orientation.z,
                            'w': point.pose.orientation.w
                        }
                    },
                    'longitudinal_velocity_mps': point.longitudinal_velocity_mps,
                    'lateral_velocity_mps': point.lateral_velocity_mps,
                    'acceleration_mps2': point.acceleration_mps2,
                    'heading_rate_rps': point.heading_rate_rps,
                    'front_wheel_angle_rad': point.front_wheel_angle_rad,
                    'rear_wheel_angle_rad': point.rear_wheel_angle_rad
                }
                for point in msg.points
            ]
        }

    # TODO: 可以在这里添加更多的插入函数
    # def insert_other_data_to_mysql(self, current_id: int):
    #     msg = self.data_manager.get_latest_message('other')
    #     if msg is None:
    #         self.get_logger().warning('No other data available to insert.')
    #         return
    #     try:
    #         connection = mysql.connector.connect(**self.config.DB_CONFIG)
    #         cursor = connection.cursor()
    #         sql = (
    #             "INSERT INTO `其他表` ("
    #             "id, other_field, create_time, modify_time, creater, modifer"
    #             ") VALUES (%s, %s, %s, %s, %s, %s)"
    #         )
    #         data = (
    #             current_id,  # id
    #             msg.some_field,                          # other_field
    #             datetime.fromtimestamp(current_id / 1000.0).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
    #             datetime.fromtimestamp(current_id / 1000.0).strftime('%Y-%m-%d %H:%M:%S.%f'),  # modify_time
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

