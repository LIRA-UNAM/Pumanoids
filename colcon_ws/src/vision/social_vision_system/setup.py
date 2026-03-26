from setuptools import setup
import os
from glob import glob

package_name = 'social_vision_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darienps8',
    maintainer_email='darienps8@gmail.com',
    description='Nodos de procesamiento de visión',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'marker_detector = social_vision_system.marker_detector:main',
        ],
    },
)