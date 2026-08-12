import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'twist_to_t1'

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
    maintainer='booster',
    maintainer_email='lir.pumas@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_t1 = twist_to_t1.twist_to_t1:main',
            'pantilt_to_t1 = twist_to_t1.pantilt_to_t1:main',
            'odom_to_tf = twist_to_t1.odom_to_tf:main'
        ],
    },
)
