import os
from glob import glob
from setuptools import setup

package_name = 'image_transfer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 确保安装模型文件到正确位置
        (os.path.join('share', package_name, 'models'), 
            glob(os.path.join('models', '*.pt'))),
        (os.path.join('share', package_name, 'yolo'), 
            glob(os.path.join('models', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lwy',
    maintainer_email='lwy@todo.todo',
    description='ROS2图像传输与YOLO检测包',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber = image_transfer.subscriber:main',
        ],
    },
)
