from setuptools import find_packages, setup

package_name = 'ros_bt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'bleak'],
    zip_safe=True,
    maintainer='ggn0',
    maintainer_email='luigi.s1994@gmail.com',
    description='Python bridge to publish topic messages over bluetooth',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "ros_bt_bridge = ros_bt_bridge.ros_bt_bridge:main"
        ],
    },
)
