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
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'sgan-models'), 
            glob(os.path.join(package_name, 'sgan', 'models', 'sgan-models', '*.pt'))),
        (os.path.join('share', package_name, 'sgan-p-models'), 
            glob(os.path.join(package_name, 'sgan', 'models', 'sgan-p-models', '*.pt'))),
        (os.path.join('share', package_name, 'trained-models'), 
            glob(os.path.join(package_name, 'crowdattn/trained_models/differential_model/checkpoints/*.pt')))
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
            'scan_receiver = lidar_process.scan_receiver:main',
            'rl_server = lidar_process.rl_server:main'
        ],
    },
)
