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
        (os.path.join('share', package_name, 'crowdattn-trained-models'), 
            glob(os.path.join(package_name, 'crowdattn/trained_models/differential_model/checkpoints/*.pt'))),
        (os.path.join('share', package_name, 'group-rl-configs'), 
            glob(os.path.join(package_name, 'group_rl', '*.yaml'))),
        (os.path.join('share', package_name, 'group-rl-configs'), 
            glob(os.path.join(package_name, 'group_rl', '*.config'))),
        # The 20-human variants of the two configs. group_rl is a checkout of
        # HiCrowd-EXPO and is gitignored here, so edits to its copies do not
        # travel to another robot -- which is exactly how the expo_orca_90deg
        # checkpoint ended up paired with a 10-human config. Keep ours in this
        # repository instead, next to the checkpoints in models/group_rl.
        (os.path.join('share', package_name, 'group-rl-configs'),
            glob(os.path.join('config', 'group_rl', '*.yaml'))
            + glob(os.path.join('config', 'group_rl', '*.config'))),
        # group_rl is checked out from HiCrowd-EXPO, so models we add ourselves live
        # here instead. rl_server.py looks both up under share/<pkg>/group-rl-models.
        (os.path.join('share', package_name, 'group-rl-models'),
            glob(os.path.join(package_name, 'group_rl', '*.zip'))
            + glob(os.path.join('models', 'group_rl', '*.zip'))),
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
