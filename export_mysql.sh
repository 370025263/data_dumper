#!/bin/bash

# 配置
DB_USER="test_user"
DB_NAME="test"
DB_PASS="password123"  # 如果不希望明文存储密码，可以去掉这一行，并在下面的命令中手动输入密码

# 生成文件名
FILE_NAME=$(date +"%Y%m%d-%H-%M-%S")_database.sql

# 导出数据库
sudo mysqldump -u $DB_USER -p$DB_PASS $DB_NAME --skip-lock-tables > $FILE_NAME

# 检查导出是否成功
if [ $? -eq 0 ]; then
    echo "数据库导出成功：$FILE_NAME"
else
    echo "数据库导出失败，请检查权限或配置。"
    exit 1
fi

# 压缩文件
gzip $FILE_NAME
echo "文件已压缩：$FILE_NAME.gz"

# 清空表
TABLES=(
    "信号灯感知表"
    "决策规划表"
    "定位表"
    "底盘表"
    "控制表"
    "激光雷达障碍物感知表"
)

for TABLE in "${TABLES[@]}"; do
    mysql -u $DB_USER -p$DB_PASS $DB_NAME -e "TRUNCATE TABLE $TABLE;"
    if [ $? -eq 0 ]; then
        echo "表 $TABLE 已清空。"
    else
        echo "清空表 $TABLE 失败，请检查权限或表是否存在。"
    fi
done

echo "数据库已导出并清空。"
