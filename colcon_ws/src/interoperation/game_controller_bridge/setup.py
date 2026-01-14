from setuptools import find_packages, setup
from glob import glob

package_name = 'game_controller_bridge'

setup(
    name=package_name,
     version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/utils/', glob('utils/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocup',
    maintainer_email='garcia.miguel.onate@gmail.com',
    description='TODO: Package description',
    license='LGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'game_controller_bridge = game_controller_bridge.game_controller_bridge:main'
        ],
    },
)
