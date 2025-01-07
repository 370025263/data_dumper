# 项目结构
stone/autoware_data_dumper/
├── data_dumper/
│   ├── data_dumper.py        # 核心逻辑：ROS 2节点订阅话题并写入MySQL
│   └── ...
├── tables_create_statements.txt  # 建表SQL示例
├── start.sh                  # 脚本：启动Data Dumper节点,开始数据录制
├── export_mysql.sh           # 脚本：导出MySQL数据并清空数据库
├── c.sh                      # 脚本：导出MYSQL表格的创建语句到'tables_create_statements.txt'
├── ...
└── README.md                 # 项目说明


# 录制数据
在启动自动驾驶系统后，输入'bash /home/casia/stone/autoware_data_dumper/start.sh'便可以开始录制。

# 数据导出
输入'bash /home/casia/stone/autoware_data_dumper/export_mysql.sh' 便可以导出(密码casia), 数据库中的历史数据会被导出到2025xxxxxx-xx-xx-xx_database.sql.gz',数据库清空。

# 查询数据
使用'sudo mysql'方式进入mysql(sudo密码casia)，然后输入'use test;'

# 数据拷贝
文彬讲我们最好是每天拷贝，那就需要你把/home/casia/stone/autoware_data_dumper/2025xxxxxx-xx-xx-xx_database.sql.gz 拷贝到自己U盘。
U盘需要插在车后备箱的工控机上，然后手动复制过去。

# 表格实现逻辑
**激光雷达障碍物感知表（列表拆分）**  
- **话题**：`/perception/object_recognition/objects` (`PredictedObjects`)  
- **拆分逻辑**：`msg.objects` 可能包含多个障碍物，每个障碍物生成一条记录。  
- **字段来源**：  
  - `vehicle_vin_code`：从 `Config.DEFAULTS` 读取写死的默认值  
  - `data_type`：写死为 `PredictedObjects`  
  - `data_date`：使用系统当前日期  
  - `create_time`：使用 `msg.header.stamp` 转为人类可读时间  
  - `timestamp`：将 `msg.header.stamp.sec` 和 `msg.header.stamp.nanosec` 拼接成字符串  
  - `obstacle_id`：用障碍物的 `object_id.uuid` 生成 UUID 字符串  
  - `type`：来自 `obstacle.classification[0].label`，若取不到则用默认值  
  - `pos_x`、`pos_y`：来自 `obstacle.kinematics.initial_pose_with_covariance.pose.position.x / y`  
  - `height`、`width`、`length`：来自 `obstacle.shape.dimensions.z / y / x`  

**信号灯感知表（列表拆分）**  
- **话题**：`/perception/traffic_light_recognition/traffic_signals` (`TrafficLightGroupArray`)  
- **拆分逻辑**：`msg.traffic_light_groups` 下的每个 group 可能包含多个 element，每个 element 生成一条记录。  
- **字段来源**：  
  - `vehicle_vin_code`：默认值  
  - `data_type`：写死为 `TrafficLightGroupArray`  
  - `data_date`：系统当前日期  
  - `create_time`：用 `msg.stamp` 转为人类可读时间  
  - `timestamp`：`msg.stamp.sec.nanosec` 形式  
  - `traffic_light_group_id`：`group.traffic_light_group_id` (目前仅返回 0)  
  - `element_id`：在同一组里，第几个灯（索引+1）  
  - `color`：`element.color`，用映射函数转换为 `RED1/AMBER2/GREEN3/WHITE4`，若取不到则 UNKNOWN  
  - `shape`：`element.shape`，映射为形状字符串，若取不到则 UNKNOWN  
  - `status`：`element.status`，映射为 `熄火/常亮/闪烁`，若取不到则 UNKNOWN  
  - `confidence`：`element.confidence`，若取不到则写默认值  

**定位表（单条写入）**  
- **话题**：  
  - `/localization/kinematic_state` (`Odometry`)：提供航向角（由四元数计算）、XYZ、GNSS 运行状态（pose.covariance[1]）、IMU 运行状态（pose.covariance[2]）  
  - `/sensing/gnss/monitor` (`NavSatFix`)：提供 `latitude`、`longitude`  
  - `/sensing/imu/imu_raw` (`Imu`)：提供 `linear_acceleration.x/y`  
- **写入时机**：上面三个话题任何一个更新时，就尝试用最新消息写表  
- **字段来源**：  
  - `vehicle_vin_code`：默认值  
  - `data_type`：写死 `Localization`  
  - `data_date`：系统当前日期  
  - `create_time`、`timestamp`：来自 `Odometry` 消息的 header.stamp  
  - `vehicle_heading_angle`：由四元数转 yaw 得到  
  - `lon`/`lat`：`NavSatFix.longitude / latitude`，若没取到则默认值  
  - `lateral_acceleration`/`longitudinal_acceleration`：`Imu.linear_acceleration.y / x`，若没取到则默认值  
  - `gnss_state`/`imu_state`：来自 `Odometry.pose.covariance[1] / [2]`，若为空则默认  
  - `x`/`y`/`z`：来自 `Odometry.pose.pose.position.x/y/z`  
  - `o_lon/o_lat/o_h`：写死为 `119.69281743/25.4804208/36.488`  

**控制表（单条写入）**  
- **话题**：  
  - 档位：`/vehicle/status/gear_status` (`GearReport`)  
  - 前轮转角：`/vehicle/status/steering_status` (`SteeringReport`)  
  - 纵向加速度：`/control/command/control_cmd` (`Control`) 中的 `longitudinal.acceleration`  
  - 转向灯：`/control/command/turn_indicators_cmd` (`TurnIndicatorsCommand`)  
  - 紧急灯：`/control/command/hazard_lights_cmd` (`HazardLightsCommand`)  
  - 自动驾驶模式指令：目前代码写死为 `0`（话题未发布）  
  - 急停：`/control/command/emergency_cmd` (`VehicleEmergencyStamped`)  
- **写入时机**：任意一个必要话题更新，就用最新数据写一条记录  
- **字段来源**：  
  - `vehicle_vin_code`：默认值  
  - `data_type`：写死为 `Control`  
  - `data_date`：系统当前日期  
  - `create_time`：用 `Control` 消息的时间戳转换  
  - `timestamp`：用 `GearReport` 消息时间戳作主键  
  - `gear`：`GearReport.report` 映射为 `1N,2D,20R,22P`  
  - `steering_tire_angle`：`SteeringReport.steering_tire_angle`  
  - `longitudinal_acceleration`：`Control.longitudinal.acceleration`  
  - `turn_light`：`TurnIndicatorsCommand.command` 映射为 `1off,2left,3right`  
  - `hazard_light`：`HazardLightsCommand.command` 映射为 `1off,2on`  
  - `drive_mode`：暂时写死为 `0`  
  - `emergency`：`VehicleEmergencyStamped.emergency` 布尔值转换为字符串  

**决策规划表（列表拆分）**  
- **话题**：`/planning/scenario_planning/trajectory` (`Trajectory`)  
- **拆分逻辑**：`msg.points` 中每个点都写入一条记录  
- **字段来源**：  
  - `vehicle_vin_code`：默认值  
  - `data_type`：写死为 `DecisionPlanning`  
  - `data_date`：系统当前日期  
  - `create_time`、`timestamp`：用 `msg.header.stamp` 转为时间字符串  
  - `turn_light` / `hazard_light`：当前在代码中用 `TurnIndicatorsCommand` 和 `HazardLightsCommand` 最新指令映射  
  - `p_id`：轨迹点序号（从 1 开始）  
  - `x/y/z`：`TrajectoryPoint.pose.position`  
  - `o_x/o_y/o_z/o_w`：`TrajectoryPoint.pose.orientation`  
  - `longitudinal_velocity_mps` / `lateral_velocity_mps` / `acceleration_mps2` / `heading_rate_rps` / `front_wheel_angle_rad`：均直接取自每个轨迹点  

**底盘表（单条写入）**  
- **话题**：  
  - 档位：`/vehicle/status/gear_status` (`GearReport`)  
  - 前轮转角：`/vehicle/status/steering_status` (`SteeringReport`)  
  - 纵向加速度：目前置空，尚无对应实现  
  - 转向灯、紧急灯：`/vehicle/status/turn_indicators_status`、`/vehicle/status/hazard_lights_status` (或合并在同一消息)  
  - 自动驾驶模式：`/vehicle/status/control_mode` (`ControlModeReport.mode`) 映射为 `1auto,4manual`  
  - 纵向、横向速度、航向角变化率：`/vehicle/status/velocity_status` (`VelocityReport`) 中的 `longitudinal_velocity`, `lateral_velocity`, `heading_rate`  
- **写入时机**：任意必要话题更新，就使用所有最新值写一条记录  
- **字段来源**：  
  - `vehicle_vin_code`：默认值  
  - `data_type`：写死为 `Chassis`  
  - `data_date`：系统当前日期  
  - `create_time`、`timestamp`：使用 `VelocityReport` 时间戳  
  - `gear`：`GearReport.report` 映射  
  - `steering_tire_angle`：`SteeringReport.steering_tire_angle`  
  - `longitudinal_acceleration`：暂时未实现  
  - `turn_light` / `hazard_light`：从相应的 status 消息取 `report` 并映射  
  - `drive_mdoe`：由 `ControlModeReport.mode` 映射  
  - `longitudinal_velocity` / `lateral_velocity` / `heading_rate`：`VelocityReport` 中对应字段，若取不到则默认  

上述各表记录的写入逻辑均参考代码里的回调函数，话题一旦发布新的消息，就会更新最新数据并写入数据库。若某些字段取不到值或未发布对应话题，则使用默认值或不写入。



# 新增或修改表格字段
确定字段需求
在新增/修改字段前，需要在 MySQL 数据库中执行相应的 ALTER TABLE 或 CREATE TABLE 语句。可以参考 tables_create_statements.txt 中的格式。

在 Config.DEFAULTS 中添加或更新默认值
如果需要新的默认值（例如当消息缺失时使用），可在 Config 类的 DEFAULTS 字典中添加对应键值对。

python
复制代码
DEFAULTS = {
    ...
    'my_new_field_default': 'SOME_DEFAULT_VALUE'
}
在回调函数或插入函数中处理新的字段

如果是已有表增加字段，需要在对应的 INSERT 或 UPDATE SQL 语句中添加该字段，并在数据元组中增加相应的取值逻辑。
如果是新的字段，可能需要从ROS消息里解析、映射或做一些运算后再插入。
更新数据库结构

确保 MySQL 中已执行了添加字段的 ALTER TABLE 语句，并与代码的 INSERT 语句保持字段顺序一致。
批量插入时，需要同时增加对应的列与数据位置。
示例：给 底盘表 中新增一个 battery_status 字段时的步骤：

在 MySQL 中执行：
sql
复制代码
ALTER TABLE `底盘表` ADD COLUMN `battery_status` VARCHAR(255) DEFAULT NULL;
在 Config.DEFAULTS 中添加默认值（如果需要）：
python
复制代码
'battery_status_default': 'UNKNOWN'
在 insert_chassis 函数的 SQL 语句和 data = (...) 中相应位置插入该字段和取值逻辑。
<a id="如何添加新的表格并实现写入逻辑"></a>

2.3 如何添加新的表格并实现写入逻辑
在数据库中创建新表
参考 tables_create_statements.txt 中已有示例编写新表的 CREATE TABLE 语句。注意字段类型、长度以及主键设置。

在 setup_subscribers 中订阅新话题
例如：

python
复制代码
self.new_topic_subscription = self.create_subscription(
    NewMsgType,                   # 新的消息类型
    '/my/new/topic',             # 新的话题名
    self.new_topic_callback,      # 回调函数
    10
)
在回调函数 new_topic_callback 中，将消息写入 self.latest_messages，如：

```python
复制代码
def new_topic_callback(self, msg: NewMsgType):
    with self.lock:
        self.latest_messages['/my/new/topic'] = msg
    self.insert_new_table()  # 立即写入数据库
```

编写插入函数 insert_new_table
参考已有的 insert_chassis 或 insert_control 等函数，获取最新消息，判断是否具备写入所需的所有话题数据，然后将组装好的数据通过 SQL 语句插入到新表中。

```python
复制代码
def insert_new_table(self):
    # 1) 从 self.latest_messages 中获取所需的话题消息
    # 2) 检查是否所有消息都存在
    # 3) 构造 SQL 语句并执行
    # 4) 提交事务 & 错误处理
```

发布相应 ROS 话题消息，观察程序是否能正确将数据写入新表。
使用 MySQL 客户端或 export_mysql.sh 验证数据内容和格式是否符合预期。
<a id="常见问题及排查"></a>

2.4 常见问题及排查
一直提示“Missing required messages for XXX table”

确认所有所需话题都在发布，查看话题名称是否正确。
检查是否因为某些话题发布频率太低，导致暂时还未收到消息。
数据库插入失败 (Data too long for column ...)

数据长度超过字段定义。可在 MySQL 中将字段改为更大长度 (e.g. VARCHAR(500))，或改用合适的数值类型。
不知道某个字段的默认值

在 Config.DEFAULTS 中添加一个适当的键值，并在插入逻辑中引用它。
查看表结构

使用 DESCRIBE <表名> 或查看 tables_create_statements.txt 了解每个字段的含义与类型。
