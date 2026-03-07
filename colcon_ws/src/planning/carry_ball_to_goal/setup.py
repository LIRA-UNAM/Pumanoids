import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'carry_ball_to_goal'

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
    maintainer_email='booster@todo.todo',
    description='Launch para llevar la pelota al arco',
    license='MIT',
    entry_points={
        'console_scripts': [
            'goal_robot_pose = carry_ball_to_goal.goal_robot_pose:main',
            'go_to_goal_pose = carry_ball_to_goal.go_to_goal_pose:main',
        ],
    },
)
