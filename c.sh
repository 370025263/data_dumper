#!/bin/bash

# 配置
DB_USER="test_user"  # 替换为你的 MySQL 用户名
DB_NAME="test"       # 替换为你的数据库名
OUTPUT_FILE="tables_create_statements.txt"  # 输出文件名

# 提示用户输入密码
read -s -p "请输入 MySQL 密码: " DB_PASS
echo

# 清空输出文件
> "$OUTPUT_FILE"

# 获取所有表名
TABLES=$(mysql -u $DB_USER -p$DB_PASS -N -B -e "SHOW TABLES FROM $DB_NAME;")

# 遍历每个表并导出创建语句
for TABLE in $TABLES; do
    echo "导出表 $TABLE 的创建语句..."
    mysql -u $DB_USER -p$DB_PASS -e "SHOW CREATE TABLE $DB_NAME.$TABLE;" >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"  # 添加空行分隔
done

echo "所有表的创建语句已导出到 $OUTPUT_FILE。"
