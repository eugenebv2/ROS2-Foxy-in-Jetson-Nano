from setuptools import setup
import os
from glob import glob

package_name = 'arm_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='bv2ci@yahoo.com.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lsc32_test_1 = arm_test.lsc32_test_1:main',
            'lsc32_single_servo_controller = arm_test.lsc32_single_servo_controller:main',
            'lsc32_multi_servo_controller = arm_test.lsc32_multi_servo_controller:main',
            'lsc32_multi_servo_controller_param = arm_test.lsc32_multi_servo_controller_param:main',
            'lsc32_command_publisher = arm_test.lsc32_command_publisher:main',
            'lsc32_multi_servo_service = arm_test.lsc32_multi_servo_service:main',
            'lsc32_service_client = arm_test.lsc32_service_client:main',
            'lsc32_yaml_service_client = arm_test.lsc32_yaml_service_client:main',
            'lsc32_yaml_service_client_loop = arm_test.lsc32_yaml_service_client_loop:main',
            'lsc32_multi_service_node = arm_test.lsc32_multi_service_node:main',
        ],
    },
)
