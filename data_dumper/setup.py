from setuptools import find_packages, setup

package_name = 'data_dumper'

setup(
    name=package_name,
    version='0.1.0',  # 使用标准化版本号
    packages=find_packages(exclude=['test']),  # 自动发现包，排除 test 文件夹
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # 注册包资源
        ('share/' + package_name, ['package.xml']),  # 包含 package.xml 文件
    ],
    install_requires=[
        'setuptools',
        'mysql-connector-python',  # 添加 MySQL 依赖
        'rclpy',  # ROS 2 的 Python 客户端
    ],
    zip_safe=True,
    maintainer='casia',  # 维护者信息
    maintainer_email='15810012030@163.com',  # 邮箱
    description='A ROS 2 package for GNSS data processing and database interaction.',
    license='Apache License 2.0',  # 推荐使用标准的开源许可证
    extras_require={
        'test': ['pytest'],  # 测试依赖项，使用现代工具支持
    },
    entry_points={
        'console_scripts': [
            'data_dumper = data_dumper.data_dumper:main',  # ROS 2 节点入口
        ],
    },
)

