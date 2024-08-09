from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_process'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='allanwangliqian',
    maintainer_email='allanwangliqian@gmail.com',
    description='This package processes lidar scans into groups and predicts groups',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_receiver = lidar_process.scan_receiver:main'
        ],
    },
)
