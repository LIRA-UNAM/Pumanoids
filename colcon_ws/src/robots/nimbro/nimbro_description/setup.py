from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nimbro_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'robots')                      ,glob('robots/*')),
        (os.path.join('share', package_name, 'mesh', 'igus_op')           ,glob('mesh/igus_op/*')),
        (os.path.join('share', package_name, 'mesh', 'igus_op_hull')      ,glob('mesh/igus_op_hull/*')),
        (os.path.join('share', package_name, 'mesh', 'nimbro_adult')      ,glob('mesh/nimbro_adult/*')),
        (os.path.join('share', package_name, 'mesh', 'nimbro_adult_hull') ,glob('mesh/nimbro_adult/*')),
        (os.path.join('share', package_name, 'mesh', 'nimbro_op')         ,glob('mesh/nimbro_op/*')),
        (os.path.join('share', package_name, 'mesh', 'nimbro_op_hull')    ,glob('mesh/nimbro_op_hull/*')),
        (os.path.join('share', package_name, 'materials')                   ,glob('materials/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
