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
