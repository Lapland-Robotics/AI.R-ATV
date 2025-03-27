from setuptools import find_packages, setup
import os

package_name = 'robot'
launch_file_path = os.path.join(os.path.dirname(__file__), 'launch')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            [
                os.path.join(launch_file_path, 'run.launch.py')
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sohan-lapinamk',
    maintainer_email='sohanjanaka101@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_debug = robot.robot_debug:main",
            "wheel_odom_publisher = robot.wheel_odom_publisher:main"
        ],
    },
)
