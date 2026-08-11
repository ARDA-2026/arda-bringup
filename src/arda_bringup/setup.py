import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arda_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web', 'static'),
            glob('web/static/*')),
        (os.path.join('share', package_name, 'data'),
            glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dabin',
    maintainer_email='dlekqls3915@gmail.com',
    description='ARDA 시스템 ROS2 브링업 패키지 — 센서 추적, 표류 예측, 웹 관제 브리지',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'tracker_node = arda_bringup.tracker_node:main',
            'drift_node = arda_bringup.drift_node:main',
            'web_bridge_node = arda_bringup.web_bridge_node:main',
        ],
    },
)
