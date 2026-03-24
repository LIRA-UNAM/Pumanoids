from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'person_fallower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='booster',
    maintainer_email='lir.pumas@gmail.com',
    description='Paquete de seguimiento de personas utilizando DeepFace.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepface_follower_node = person_fallower.deepface_follower_node:main',
            'person_fallower = person_fallower.person_fallower:main'
        ],
    },
)