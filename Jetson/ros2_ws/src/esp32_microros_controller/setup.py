from setuptools import find_packages, setup

package_name = 'esp32_microros_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "esp32_controller_node = esp32_microros_controller.esp32_microros_controller:main"
        ],
    },
)
