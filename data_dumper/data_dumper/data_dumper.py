#!/usr/bin/env python3
"""
README

功能：
1. 订阅各个ROS话题。
2. 当单个话题有新消息时，立即将其数据写入对应的MySQL表。
3. 对于由多个话题组成的表（定位表、控制表），任何一个构成话题有新消息时，立即将整个表的数据写入MySQL，使用所有相关话题的最新数据。
4. 六个表:"激光雷达障碍物感知表，信号灯感知表，定位表，控制表，决策规划表，底盘表".
5. 使用数据库连接池优化MySQL连接管理。

使用方法：
1. 确保已安装必要的Python库：
    ```bash
    pip install mysql-connector-python
    ```
2. 根据实际情况修改Config类中的数据库配置和默认值。
3. 确保MySQL中已创建所需的六个表，且字段类型与代码中的插入逻辑一致。
4. 启动ROS 2节点：
    ```bash
    bash start.sh
    ```
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray
from autoware_vehicle_msgs.msg import ControlModeReport, HazardLightsReport, TurnIndicatorsReport
from autoware_vehicle_msgs.msg import (
    VelocityReport,
    GearReport,
    SteeringReport,
    TurnIndicatorsCommand,
    HazardLightsCommand
)
from autoware_control_msgs.msg import Control
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
import mysql.connector
from mysql.connector import pooling
from datetime import datetime
from typing import Any, Dict
import math,uuid
import threading

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

    # 创建数据库连接池
    DB_POOL = pooling.MySQLConnectionPool(
        pool_name="mypool",
        pool_size=10,
        **DB_CONFIG
    )

    # Default values
    DEFAULTS = {
        'vehicle_vin_code_default': 123456,  # 根据实际车辆 VIN 代码调整
        'data_type_predicted_objects': 'PredictedObjects',
        'data_type_traffic_light_groups': 'TrafficLightGroupArray',
        'data_type_localization': 'Localization',
        'data_type_control': 'Control',
        'data_type_decision_planning': 'DecisionPlanning',
        'data_type_chassis': 'Chassis',
        # 激光雷达障碍物感知表的默认值
        'type_default': 'UNKNOWN',
        # 信号灯感知表的默认值
        'color_default': 'UNKNOWN',
        'shape_default': 'UNKNOWN',
        'status_default': 'UNKNOWN',
        'confidence_default': '0.0',
        # 定位表的默认值
        'vehicle_heading_angle_default': '0.0',
        'lon_default': '0.0',
        'lat_default': '0.0',
        'lateral_acceleration_default': '0.0',
        'longitudinal_acceleration_default': '0.0',
        'gnss_state_default': 'UNKNOWN',
        'imu_state_default': 'UNKNOWN',
        'x_default': '0.0',
        'y_default': '0.0',
        'z_default': '0.0',
        'o_lon_default': '119.69281743',
        'o_lat_default': '25.4804208',
        'o_h_default': '36.488',
        # 控制表的默认值
        'gear_default': 'N',
        'steering_tire_angle_default': '0.0',
        'turn_light_default': 'OFF',
        'hazard_light_default': 'OFF',
        'drive_mode_default': 'MANUAL',
        'emergency_default': 'OFF',
        # 决策规划表的默认值
        'p_id_default': '0',
        # 底盘表的默认值
        'longitudinal_velocity_default': '0.0',
        'lateral_velocity_default': '0.0',
        'heading_rate_default': '0.0',
    }

class Listener(Node):
    """
    ROS 2 Node that listens to various data topics and inserts data into MySQL.
    Designed for immediate insertion upon message reception.
    """
    def __init__(self):
        super().__init__('data_dumper')  # 初始化节点，节点名称为 'data_dumper'
        self.config = Config()
        self.latest_messages: Dict[str, Any] = {}
        self.lock = threading.Lock()
        self.setup_subscribers()

    def setup_subscribers(self):
        """
        Sets up subscribers for various ROS topics.
        """
        # 激光雷达障碍物感知表 Subscriber
        self.predicted_objects_subscription = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.predicted_objects_callback,
            10)
        self.predicted_objects_subscription  # 防止未使用变量的警告

        # 信号灯感知表 Subscriber
        self.traffic_light_groups_subscription = self.create_subscription(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals',
            self.traffic_light_groups_callback,
            10)
        self.traffic_light_groups_subscription  # 防止未使用变量的警告

        # 定位表 Subscribers
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odometry_callback,
            10)
        self.odometry_subscription

        self.gnss_subscription = self.create_subscription(
            NavSatFix,
            '/sensing/gnss/monitor',
            self.gnss_callback,
            10)
        self.gnss_subscription

        self.imu_subscription = self.create_subscription(
            Imu,
            '/sensing/imu/imu_raw',
            self.imu_callback,
            10)
        self.imu_subscription

        # 控制表 Subscribers
        self.gear_status_subscription = self.create_subscription(
            GearReport,
            '/vehicle/status/gear_status',
            self.gear_status_callback,
            10)
        self.gear_status_subscription

        self.steering_status_subscription = self.create_subscription(
            SteeringReport,
            '/vehicle/status/steering_status',
            self.steering_status_callback,
            10)
        self.steering_status_subscription

        self.control_cmd_subscription = self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self.control_cmd_callback,
            10)
        self.control_cmd_subscription

        self.turn_indicators_cmd_subscription = self.create_subscription(
            TurnIndicatorsCommand,
            '/control/command/turn_indicators_cmd',
            self.turn_indicators_cmd_callback,
            10)
        self.turn_indicators_cmd_subscription

        self.hazard_lights_cmd_subscription = self.create_subscription(
            HazardLightsCommand,
            '/control/command/hazard_lights_cmd',
            self.hazard_lights_cmd_callback,
            10)
        self.hazard_lights_cmd_subscription

        # 决策规划表 Subscriber
        self.planning_trajectory_subscription = self.create_subscription(
            Trajectory,
            '/planning/scenario_planning/trajectory',
            self.planning_trajectory_callback,
            10)
        self.planning_trajectory_subscription

        # 底盘表 Subscriber
        self.velocity_status_subscription = self.create_subscription(
            VelocityReport,  # 替换为实际的消息类型
            '/vehicle/status/velocity_status',
            self.velocity_status_callback,
            10)
        self.velocity_status_subscription
        
        self.control_mode_subscription = self.create_subscription(
            ControlModeReport,
            '/vehicle/status/control_mode',
            self.control_mode_callback,
            10
        )
        self.control_mode_subscription

        self.turn_light_status_subscription = self.create_subscription(
            TurnIndicatorsReport,
            '/vehicle/status/turn_indicators_status',
            self.turn_light_status_callback,
            10
        )
        self.turn_light_status_subscription

        self.hazard_lights_status_subscription = self.create_subscription(
            HazardLightsReport,
            '/vehicle/status/hazard_lights_status',
            self.hazard_lights_status_callback,
            10
        )
        self.hazard_lights_status_subscription
    # --------------- 回调函数和写入函数 ---------------

    # 激光雷达障碍物感知表 Callback
    def predicted_objects_callback(self, msg: PredictedObjects):
        """
        Callback function for Predicted Objects data. Inserts data into MySQL.
        Splits the list of obstacles into multiple records.
        """
        with self.lock:
            self.latest_messages['/perception/object_recognition/objects'] = msg

        if not msg.objects:
            self.get_logger().warning('Received PredictedObjects message with no objects.')
            return

        try:
            connection = self.config.DB_POOL.get_connection()
            cursor = connection.cursor()

            sql = (
                "INSERT INTO `激光雷达障碍物感知表` ("
                "vehicle_vin_code, data_type, data_date, create_time, timestamp, obstacle_id, type, "
                "pos_x, pos_y, height, width, length"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
            )

            for obstacle in msg.objects:
                obstacle_id = str(uuid.UUID(bytes=bytes(obstacle.object_id.uuid)))
                data = (
                    Config.DEFAULTS['vehicle_vin_code_default'],  # vehicle_vin_code
                    Config.DEFAULTS['data_type_predicted_objects'],  # data_type
                    datetime.now().date(),  # data_date
                    datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                    f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",  # timestamp
                    obstacle_id,  # obstacle_id
                    obstacle.classification[0].label if obstacle.classification[0].label else Config.DEFAULTS['type_default'],  # type. 
                    # TODO:表中添加一个obstacle.classification.probability
                    # ObjectClassification[] classification
                    str(obstacle.kinematics.initial_pose_with_covariance.pose.position.x),  # pos_x
                    str(obstacle.kinematics.initial_pose_with_covariance.pose.position.y),  # pos_y
                    str(obstacle.shape.dimensions.z),  # height
                    str(obstacle.shape.dimensions.y),  # width
                    str(obstacle.shape.dimensions.x),  # length
                )
                cursor.execute(sql, data)

            connection.commit()
            self.get_logger().info('PredictedObjects data inserted into MySQL.')

        except mysql.connector.Error as err:
            self.get_logger().error(f"Failed to write PredictedObjects data to MySQL: {err}")

        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()

    # 信号灯感知表 Callback
    def traffic_light_groups_callback(self, msg: TrafficLightGroupArray):
        """
        Callback function for Traffic Light Groups data. Inserts data into MySQL.
        Splits the list of traffic lights into multiple records.
        """
        with self.lock:
            self.latest_messages['/perception/traffic_light_recognition/traffic_signals'] = msg

        if not msg.traffic_light_groups:
            print(msg.traffic_light_groups)
            self.get_logger().warning('Received TrafficLightGroupArray message with no groups.')
            return

        try:
            connection = self.config.DB_POOL.get_connection()
            cursor = connection.cursor()

            sql = (
                "INSERT INTO `信号灯感知表` ("
                "vehicle_vin_code, data_type, data_date, create_time, timestamp, traffic_light_group_id, "
                "element_id, color, shape, status, confidence"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
            )

            for group in msg.traffic_light_groups:
                group_id = group.traffic_light_group_id
                for idx, element in enumerate(group.elements):
                    data = (
                        Config.DEFAULTS['vehicle_vin_code_default'],  # vehicle_vin_code
                        Config.DEFAULTS['data_type_traffic_light_groups'],  # data_type
                        datetime.now().date(),  # data_date
                        datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec * 1e-9).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                        f"{msg.stamp.sec}.{msg.stamp.nanosec}",  # timestamp
                        str(group_id),  # traffic_light_group_id
                        str(idx + 1),  # element_id (第几个)
                        self.map_color(element.color),  # color
                        self.map_shape(element.shape),  # shape
                        self.map_status(element.status),  # status
                        str(element.confidence) if element.confidence else Config.DEFAULTS['confidence_default']  # confidence
                    )
                    cursor.execute(sql, data)

            connection.commit()
            self.get_logger().info('TrafficLightGroupArray data inserted into MySQL.')

        except mysql.connector.Error as err:
            self.get_logger().error(f"Failed to write TrafficLightGroupArray data to MySQL: {err}")

        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()

    # 定位表 Callback
    def odometry_callback(self, msg: Odometry):
        """
        Callback function for Odometry data. Updates latest message and inserts data into MySQL.
        Triggers write for '定位表'.
        """
        with self.lock:
            self.latest_messages['/localization/kinematic_state'] = msg
        self.insert_localization()

    def gnss_callback(self, msg: NavSatFix):
        """
        Callback function for GNSS data. Updates latest message and inserts data into MySQL.
        Triggers write for '定位表'.
        """
        with self.lock:
            self.latest_messages['/sensing/gnss/monitor'] = msg
        self.insert_localization()

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU data. Updates latest message and inserts data into MySQL.
        Triggers write for '定位表'.
        """
        with self.lock:
            self.latest_messages['/sensing/imu/imu_raw'] = msg
        self.insert_localization()

    def insert_localization(self):
        """
        Inserts data into '定位表' using the latest Odometry, GNSS, and IMU messages.
        """
        with self.lock:
            odometry_msg: Odometry = self.latest_messages.get('/localization/kinematic_state')
            gnss_msg: NavSatFix = self.latest_messages.get('/sensing/gnss/monitor')
            imu_msg: Imu = self.latest_messages.get('/sensing/imu/imu_raw')

        if not all([odometry_msg, gnss_msg, imu_msg]):
            self.get_logger().warning('Insufficient Localization data available to insert.')
            return

        try:
            yaw = self.quaternion_to_yaw(
                odometry_msg.pose.pose.orientation.x,
                odometry_msg.pose.pose.orientation.y,
                odometry_msg.pose.pose.orientation.z,
                odometry_msg.pose.pose.orientation.w
            )

            connection = self.config.DB_POOL.get_connection()
            cursor = connection.cursor()

            sql = (
                "INSERT INTO `定位表` ("
                "vehicle_vin_code, data_type, data_date, create_time, timestamp, vehicle_heading_angle, "
                "lon, lat, lateral_acceleration, longitudinal_acceleration, gnss_state, imu_state, "
                "x, y, z, o_lon, o_lat, o_h"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) "
                "ON DUPLICATE KEY UPDATE "
                "vehicle_vin_code=VALUES(vehicle_vin_code), "
                "data_type=VALUES(data_type), "
                "data_date=VALUES(data_date), "
                "create_time=VALUES(create_time), "
                "vehicle_heading_angle=VALUES(vehicle_heading_angle), "
                "lon=VALUES(lon), "
                "lat=VALUES(lat), "
                "lateral_acceleration=VALUES(lateral_acceleration), "
                "longitudinal_acceleration=VALUES(longitudinal_acceleration), "
                "gnss_state=VALUES(gnss_state), "
                "imu_state=VALUES(imu_state), "
                "x=VALUES(x), "
                "y=VALUES(y), "
                "z=VALUES(z), "
                "o_lon=VALUES(o_lon), "
                "o_lat=VALUES(o_lat), "
                "o_h=VALUES(o_h)"
            )

            data = (
                Config.DEFAULTS['vehicle_vin_code_default'],  # vehicle_vin_code
                Config.DEFAULTS['data_type_localization'],  # data_type
                datetime.now().date(),  # data_date
                datetime.fromtimestamp(odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec * 1e-9).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                f"{odometry_msg.header.stamp.sec}.{odometry_msg.header.stamp.nanosec}",  # timestamp
                str(yaw),  # vehicle_heading_angle
                str(gnss_msg.longitude) if gnss_msg.longitude is not None else Config.DEFAULTS['lon_default'],  # lon
                str(gnss_msg.latitude) if gnss_msg.latitude is not None else Config.DEFAULTS['lat_default'],    # lat
                str(imu_msg.linear_acceleration.y) if imu_msg.linear_acceleration.y is not None else Config.DEFAULTS['lateral_acceleration_default'],  # lateral_acceleration
                str(imu_msg.linear_acceleration.x) if imu_msg.linear_acceleration.x is not None else Config.DEFAULTS['longitudinal_acceleration_default'],  # longitudinal_acceleration
                odometry_msg.pose.covariance[1] if odometry_msg.pose.covariance[1] else Config.DEFAULTS['gnss_state_default'],  # gnss_state
                odometry_msg.pose.covariance[2] if odometry_msg.pose.covariance[2] else Config.DEFAULTS['imu_state_default'],  # imu_state
                str(odometry_msg.pose.pose.position.x),  # x
                str(odometry_msg.pose.pose.position.y),  # y
                str(odometry_msg.pose.pose.position.z),  # z
                Config.DEFAULTS['o_lon_default'],  # o_lon
                Config.DEFAULTS['o_lat_default'],  # o_lat
                Config.DEFAULTS['o_h_default']    # o_h
            )

            cursor.execute(sql, data)
            connection.commit()
            self.get_logger().info('定位表 (Localization) data inserted into MySQL.')

        except mysql.connector.Error as err:
            self.get_logger().error(f"Failed to write Localization data to MySQL: {err}")

        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()

    # 底盘表 Callbacks
    def velocity_status_callback(self, msg: VelocityReport):
        """
        Callback function for Velocity Status data. Inserts data into MySQL.
        Triggers write for '底盘表'.
        """
        with self.lock:
            self.latest_messages['/vehicle/status/velocity_status'] = msg
        self.insert_chassis()
     
    def control_mode_callback(self, msg: ControlModeReport):
        """
        Callback function for Control Mode data. Updates latest message and inserts data into MySQL.
        Triggers write for '底盘表'.
        """
        with self.lock:
            self.latest_messages['/vehicle/status/control_mode'] = msg
        self.insert_chassis()

    def turn_light_status_callback(self, msg: TurnIndicatorsReport):
        """
        Callback function for Hazard Lights Status data. Updates latest message and inserts data into MySQL.
        Triggers write for '底盘表'.
        """
        with self.lock:
            self.latest_messages['/vehicle/status/turn_indicators_status'] = msg
        self.insert_chassis()

    def hazard_lights_status_callback(self, msg: HazardLightsReport):
        """
        Callback function for Hazard Lights Status data. Updates latest message and inserts data into MySQL.
        Triggers write for '底盘表'.
        """
        with self.lock:
            self.latest_messages['/vehicle/status/hazard_lights_status'] = msg
        self.insert_chassis()
    
    # 控制表 Callbacks
    def gear_status_callback(self, msg: GearReport):
        """
        Callback function for Gear Status data. Updates latest message and inserts data into MySQL.
        Triggers write for '控制表'.
        """
        with self.lock:
            self.latest_messages['/vehicle/status/gear_status'] = msg
        self.insert_control()

    def steering_status_callback(self, msg: SteeringReport):
        """
        Callback function for Steering Status data. Updates latest message and inserts data into MySQL.
        Triggers write for '控制表'.
        """
        with self.lock:
            self.latest_messages['/vehicle/status/steering_status'] = msg
        self.insert_control()

    def control_cmd_callback(self, msg: Control):
        """
        Callback function for Control Command data. Updates latest message and inserts data into MySQL.
        Triggers write for '控制表'.
        """
        with self.lock:
            self.latest_messages['/control/command/control_cmd'] = msg
        self.insert_control()

    def turn_indicators_cmd_callback(self, msg: TurnIndicatorsCommand):
        """
        Callback function for Turn Indicators Command data. Updates latest message and inserts data into MySQL.
        Triggers write for '控制表'.
        """
        with self.lock:
            self.latest_messages['/control/command/turn_indicators_cmd'] = msg
        self.insert_control()

    def hazard_lights_cmd_callback(self, msg: HazardLightsCommand):
        """
        Callback function for Hazard Lights Command data. Updates latest message and inserts data into MySQL.
        Triggers write for '控制表'.
        """
        with self.lock:
            self.latest_messages['/control/command/hazard_lights_cmd'] = msg
        self.insert_control()

    def emergency_cmd_callback(self, msg: VehicleEmergencyStamped):
        """
        Callback function for Emergency Command data. Updates latest message and inserts data into MySQL.
        Triggers write for '控制表'.
        """
        with self.lock:
            self.latest_messages['/control/command/emergency_cmd'] = msg
        self.insert_control()
    
    # 底盘表插入
    def insert_chassis(self):
        """
        Inserts data into '底盘表' using the latest VelocityReport and other related messages.
        """
        with self.lock:
            velocity_msg: VelocityReport = self.latest_messages.get('/vehicle/status/velocity_status')
            gear_msg: GearReport = self.latest_messages.get('/vehicle/status/gear_status')
            steering_msg: SteeringReport = self.latest_messages.get('/vehicle/status/steering_status')
            turn_light_status_msg: TurnIndicatorsReport = self.latest_messages.get('/vehicle/status/turn_indicators_status')
            hazard_lights_status_msg: HazardLightsReport = self.latest_messages.get('/vehicle/status/hazard_lights_status')
            control_mode_msg: ControlModeReport = self.latest_messages.get('/vehicle/status/control_mode')
        
        missing_msgs = []
        if not velocity_msg:
            missing_msgs.append('/vehicle/status/velocity_status')
        if not gear_msg:
            missing_msgs.append('/vehicle/status/gear_status')
        if not steering_msg:
            missing_msgs.append('/vehicle/status/steering_status')
        if not turn_light_status_msg:
            missing_msgs.append('/vehicle/status/turn_indicators_status')
        if not hazard_lights_status_msg:
            missing_msgs.append('/vehicle/status/hazard_lights_status')
        if not control_mode_msg:
            missing_msgs.append('/vehicle/status/control_mode')
        
        if missing_msgs:
            self.get_logger().warning(f'Missing required messages for Chassis table: {", ".join(missing_msgs)}')
            return
        
        try:
            connection = self.config.DB_POOL.get_connection()
            cursor = connection.cursor()
        
            sql = (
                "INSERT INTO `底盘表` ("
                "vehicle_vin_code, data_type, data_date, create_time, timestamp, gear, steering_tire_angle, "
                "longitudinal_acceleration, turn_light, hazard_light, drive_mode, longitudinal_velocity, "
                "lateral_velocity, heading_rate"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) "
                "ON DUPLICATE KEY UPDATE "
                "gear=VALUES(gear), "
                "steering_tire_angle=VALUES(steering_tire_angle), "
                "longitudinal_acceleration=VALUES(longitudinal_acceleration), "
                "turn_light=VALUES(turn_light), "
                "hazard_light=VALUES(hazard_light), "
                "drive_mode=VALUES(drive_mode), "
                "longitudinal_velocity=VALUES(longitudinal_velocity), "
                "lateral_velocity=VALUES(lateral_velocity), "
                "heading_rate=VALUES(heading_rate)"
            )
        
            # 自动驾驶模式指令，根据 ControlModeReport 映射
            if control_mode_msg.mode == 1:
                drive_mode = 'auto'
            elif control_mode_msg.mode == 4:
                drive_mode = 'manual'
            else:
                drive_mode = Config.DEFAULTS['drive_mode_default']
        
            # 急停指令
            emergency = 'ON' if getattr(hazard_lights_status_msg, 'emergency', False) else Config.DEFAULTS['emergency_default']
        
            # 时间戳使用 VelocityReport 的时间
            timestamp_str = f"{velocity_msg.header.stamp.sec}.{velocity_msg.header.stamp.nanosec}"
        
            # 挡位映射
            gear_map = {
                1: 'N',
                2: 'D',
                20: 'R',
                22: 'P'
            }
            gear = gear_map.get(gear_msg.report, Config.DEFAULTS['gear_default'])
        
            data = (
                Config.DEFAULTS['vehicle_vin_code_default'],  # vehicle_vin_code
                Config.DEFAULTS['data_type_chassis'],         # data_type
                datetime.now().date(),                       # data_date
                datetime.fromtimestamp(velocity_msg.header.stamp.sec + velocity_msg.header.stamp.nanosec * 1e-9).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                timestamp_str,                               # timestamp
                gear,                                        # gear
                str(steering_msg.steering_tire_angle) if steering_msg.steering_tire_angle is not None else Config.DEFAULTS['steering_tire_angle_default'],  # steering_tire_angle
                Config.DEFAULTS['longitudinal_acceleration_default'],  # longitudinal_acceleration (暂时未实现，加默认值)
                self.map_turn_light(turn_light_status_msg.report),  # turn_light (如果没有对应的话题，可设为默认值或从其他地方获取)
                self.map_hazard_light(hazard_lights_status_msg.report),  # hazard_light
                drive_mode,                                  # drive_mode
                str(velocity_msg.longitudinal_velocity) if hasattr(velocity_msg, 'longitudinal_velocity') else Config.DEFAULTS['longitudinal_velocity_default'],  # longitudinal_velocity
                str(velocity_msg.lateral_velocity) if hasattr(velocity_msg, 'lateral_velocity') else Config.DEFAULTS['lateral_velocity_default'],             # lateral_velocity
                str(velocity_msg.heading_rate) if hasattr(velocity_msg, 'heading_rate') else Config.DEFAULTS['heading_rate_default'],                         # heading_rate
            )
        
            cursor.execute(sql, data)
            connection.commit()
            self.get_logger().info('底盘表 (Chassis) data inserted into MySQL.')
        
        except mysql.connector.Error as err:
            self.get_logger().error(f"Failed to write Chassis data to MySQL: {err}")
        
        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()

    def insert_control(self):
        """
        Inserts data into '控制表' using the latest GearReport, SteeringReport, Control, TurnIndicatorsCommand,
        HazardLightsCommand, and VehicleEmergencyStamped messages.
        """
        with self.lock:
            gear_msg: GearReport = self.latest_messages.get('/vehicle/status/gear_status')
            steering_msg: SteeringReport = self.latest_messages.get('/vehicle/status/steering_status')
            control_cmd_msg: Control = self.latest_messages.get('/control/command/control_cmd')
            turn_light_msg: TurnIndicatorsCommand = self.latest_messages.get('/control/command/turn_indicators_cmd')
            hazard_light_msg: HazardLightsCommand = self.latest_messages.get('/control/command/hazard_lights_cmd')
            emergency_msg: VehicleEmergencyStamped = self.latest_messages.get('/control/command/emergency_cmd')

        if not all([gear_msg, steering_msg, control_cmd_msg]):
            self.get_logger().warning('Missing GearReport, SteeringReport, or Control Command data for Control table.')
            return

        try:
            connection = self.config.DB_POOL.get_connection()
            cursor = connection.cursor()

            sql = (
                "INSERT INTO `控制表` ("
                "vehicle_vin_code, data_type, data_date, create_time, timestamp, gear, steering_tire_angle, "
                "longitudinal_acceleration, turn_light, hazard_light, drive_mode, emergency"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
                "ON DUPLICATE KEY UPDATE "
                "gear=VALUES(gear), "
                "steering_tire_angle=VALUES(steering_tire_angle), "
                "longitudinal_acceleration=VALUES(longitudinal_acceleration), "
                "turn_light=VALUES(turn_light), "
                "hazard_light=VALUES(hazard_light), "
                "drive_mode=VALUES(drive_mode), "
                "emergency=VALUES(emergency)"
                            )

            # 自动驾驶模式指令，目前没有发布，写死为 '0'
            drive_mode = Config.DEFAULTS['drive_mode_default']

            # 急停指令
            emergency = emergency_msg.emergency if emergency_msg and emergency_msg.emergency else Config.DEFAULTS['emergency_default']

            timestamp_str = f"{gear_msg.stamp.sec}.{gear_msg.stamp.nanosec}"
            
            # 挡位映射
            gear_map = {
                1: 'N',
                2: 'D',
                20: 'R',
                22: 'P'
            }
            gear = gear_map.get(gear_msg.report, Config.DEFAULTS['gear_default'])

            data = (
                Config.DEFAULTS['vehicle_vin_code_default'],  # vehicle_vin_code
                Config.DEFAULTS['data_type_control'],  # data_type
                datetime.now().date(),  # data_date
                datetime.fromtimestamp(control_cmd_msg.stamp.sec + control_cmd_msg.stamp.nanosec * 1e-9).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                timestamp_str,  # timestamp
                str(gear),  # gear
                str(steering_msg.steering_tire_angle) if steering_msg.steering_tire_angle is not None else Config.DEFAULTS['steering_tire_angle_default'],  # steering_tire_angle
                str(control_cmd_msg.longitudinal.acceleration) if control_cmd_msg.longitudinal.acceleration is not None else Config.DEFAULTS['longitudinal_acceleration_default'],  # longitudinal_acceleration
                self.map_turn_light(turn_light_msg.command) if turn_light_msg else Config.DEFAULTS['turn_light_default'],  # turn_light
                self.map_hazard_light(hazard_light_msg.command) if hazard_light_msg else Config.DEFAULTS['hazard_light_default'],  # hazard_light
                drive_mode,  # drive_mode
                emergency  # emergency
            )

            cursor.execute(sql, data)
            connection.commit()
            self.get_logger().info('控制表 (Control) data inserted into MySQL.')

        except mysql.connector.Error as err:
            self.get_logger().error(f"Failed to write Control data to MySQL: {err}")

        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()

    # 决策规划表 Callback
    def planning_trajectory_callback(self, msg: Trajectory):
        """
        Callback function for Planning Trajectory data. Inserts data into MySQL.
        Splits the list of trajectory points into multiple records.
        """
        with self.lock:
            self.latest_messages['/planning/scenario_planning/trajectory'] = msg

        if not msg.points:
            self.get_logger().warning('Received Trajectory message with no points.')
            return

        try:
            connection = self.config.DB_POOL.get_connection()
            cursor = connection.cursor()

            sql = (
                "INSERT INTO `决策规划表` ("
                "vehicle_vin_code, data_type, data_date, create_time, timestamp, turn_light, hazard_light, "
                "p_id, x, y, z, o_x, o_y, o_z, o_w, longitudinal_velocity_mps, lateral_velocity_mps, "
                "acceleration_mps2, heading_rate_rps, front_wheel_angle_rad"
                ") VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
            )

            # 获取转向灯和应急灯状态
            with self.lock:
                turn_light_msg: TurnIndicatorsCommand = self.latest_messages.get('/control/command/turn_indicators_cmd')
                hazard_light_msg: HazardLightsCommand = self.latest_messages.get('/control/command/hazard_lights_cmd')

            turn_light = self.map_turn_light(turn_light_msg.command) if turn_light_msg else Config.DEFAULTS['turn_light_default']
            hazard_light = self.map_hazard_light(hazard_light_msg.command) if hazard_light_msg else Config.DEFAULTS['hazard_light_default']

            timestamp_str = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}"

            for idx, point in enumerate(msg.points):
                p_id = str(idx + 1)  # 轨迹点id（第几个）
                pose = point.pose
                data = (
                    Config.DEFAULTS['vehicle_vin_code_default'],  # vehicle_vin_code
                    Config.DEFAULTS['data_type_decision_planning'],  # data_type
                    datetime.now().date(),  # data_date
                    datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9).strftime('%Y-%m-%d %H:%M:%S.%f'),  # create_time
                    timestamp_str,  # timestamp
                    turn_light,  # turn_light
                    hazard_light,  # hazard_light
                    p_id,  # p_id
                    str(pose.position.x),  # x
                    str(pose.position.y),  # y
                    str(pose.position.z),  # z
                    str(pose.orientation.x),  # o_x
                    str(pose.orientation.y),  # o_y
                    str(pose.orientation.z),  # o_z
                    str(pose.orientation.w),  # o_w
                    str(point.longitudinal_velocity_mps),  # longitudinal_velocity_mps
                    str(point.lateral_velocity_mps),  # lateral_velocity_mps
                    str(point.acceleration_mps2),  # acceleration_mps2
                    str(point.heading_rate_rps),  # heading_rate_rps
                    str(point.front_wheel_angle_rad)  # front_wheel_angle_rad
                )
                cursor.execute(sql, data)

            connection.commit()
            self.get_logger().info('决策规划表 (Decision Planning) data inserted into MySQL.')

        except mysql.connector.Error as err:
            self.get_logger().error(f"Failed to write Decision Planning data to MySQL: {err}")

        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()

    # ----------------- 辅助函数 -----------------
    def generate_obstacle_id(self, uuid_bytes: bytes) -> str:
        """
        Generates a UUID string from bytes.

        :param uuid_bytes: UUID bytes from obstacle.object_id.uuid
        :return: String representation of UUID
        """
        try:
            # 假设 uuid_bytes 是16字节
            if len(uuid_bytes) != 16:
                raise ValueError("UUID bytes length is not 16.")
            obstacle_uuid = uuid.UUID(bytes=bytes(uuid_bytes))
            return str(obstacle_uuid)
        except Exception as e:
            self.get_logger().error(f"Failed to generate obstacle_id from UUID bytes: {e}")
            return '00000000-0000-0000-0000-000000000000'  # 默认UUID
     
    # 映射函数
    def map_color(self, color_code: int) -> str:
        """
        Maps color code to string representation.
        """
        color_map = {
            1: 'RED1',
            2: 'AMBER2',
            3: 'GREEN3',
            4: 'WHITE4',
            0: 'UNKNOWN'
        }
        return color_map.get(color_code, 'UNKNOWN')

    def map_shape(self, shape_code: int) -> str:
        """
        Maps shape code to string representation.
        """
        shape_map = {
            1: '圆形',
            2: '左箭头',
            3: '右箭头',
            4: '上箭头',
            5: '上左箭头',
            6: '上右箭头',
            7: '下箭头',
            8: '下左箭头',
            9: '下右箭头',
            10: '十字',
            0: 'UNKNOWN'
        }
        return shape_map.get(shape_code, 'UNKNOWN')

    def map_status(self, status_code: int) -> str:
        """
        Maps status code to string representation.
        """
        status_map = {
            1: '熄火',
            2: '常亮',
            3: '闪烁',
            0: 'UNKNOWN'
        }
        return status_map.get(status_code, 'UNKNOWN')

    def map_turn_light(self, command: int) -> str:
        """
        Maps turn light command to string representation.
        """
        turn_light_map = {
            1: 'OFF',
            2: 'LEFT',
            3: 'RIGHT'
        }
        return turn_light_map.get(command, 'UNKNOWN')

    def map_hazard_light(self, command: int) -> str:
        """
        Maps hazard light command to string representation.
        """
        hazard_light_map = {
            1: 'OFF',
            2: 'ON'
        }
        return hazard_light_map.get(command, 'UNKNOWN')

    def quaternion_to_yaw(self, x, y, z, w) -> float:
        """
        Converts a quaternion into a yaw angle (in radians).

        :param x: 四元数的 x 分量
        :param y: 四元数的 y 分量
        :param z: 四元数的 z 分量
        :param w: 四元数的 w 分量
        :return: 航向角 (Yaw)，单位为弧度
        """
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

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
    listener = Listener()  # 创建节点实例
    try:
        rclpy.spin(listener)  # 保持节点运行，等待消息
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()  # 销毁节点
        rclpy.shutdown()  # 关闭 ROS 2 Python 客户端库

if __name__ == '__main__':
    main()

