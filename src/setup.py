from setuptools import find_packages, setup

package_name = 'ximu3_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'sensor_msgs', 'std_msgs'],
    zip_safe=True,
    maintainer='Xiaobin Xiong',
    maintainer_email='well.robotics@gmail.com',
    description='ros2 imu3 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu3 = ximu3_ros2.ximu3_ros2:main'
        ],
    },
)
