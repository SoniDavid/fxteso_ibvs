import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'quad_cam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('lib', package_name), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SoniDavid',
    maintainer_email='losfansdepalomino@proton.me',
    description='Raspberry Pi Camera Module 3 Wide publisher for the fxteso_ibvs stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = quad_cam.camera_node:main',
            'probe_latency = quad_cam.probe_latency:main',
            'probe_detect = quad_cam.probe_detect:main',
            'probe_bench = quad_cam.probe_bench:main',
            'bench_feeder = quad_cam.bench_feeder:main',
        ],
    },
)
