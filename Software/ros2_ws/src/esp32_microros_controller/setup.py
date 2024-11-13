from setuptools import find_packages, setup
import os

package_name = 'esp32_microros_controller'
launch_file_path = os.path.join(os.path.dirname(__file__), 'launch', 'run.launch.py')


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            [launch_file_path]),  
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
            "esp32_controller_node = esp32_microros_controller.esp32_microros_controller:main",
            "data_collector = esp32_microros_controller.data_collector:main"
        ],
    },
)
