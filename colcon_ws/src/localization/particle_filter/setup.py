from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'particle_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruthmced',
    maintainer_email='ruthmced@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'map_node = particle_filter.map:main',
        'mirror_map_node = particle_filter.mirror_map:main',
        'mcl_node=particle_filter.mcl_node:main',
        'detector = particle_filter.detector_node:main',
        'bridge_odom = particle_filter.bridge_odom:main',
        ],
    },
    )
