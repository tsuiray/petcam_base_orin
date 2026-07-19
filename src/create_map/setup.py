from setuptools import setup
import os
from glob import glob

package_name = 'create_map'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cheuk Ming TSUI',
    maintainer_email='tsuiming2@gmail.com',
    description='IMU path mapping from ESP32-S3 to Orin base station',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_odometry = create_map.imu_odometry_node:main',
            'map_viewer = create_map.map_viewer_node:main',
            'mock_imu = create_map.mock_imu_publisher:main',
        ],
    },
)
