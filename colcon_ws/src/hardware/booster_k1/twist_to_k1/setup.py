from setuptools import find_packages, setup

package_name = 'twist_to_k1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='angel',
    maintainer_email='mglp1595@gmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_to_k1 = twist_to_k1.twist_to_k1:main',
            'pantilt_to_k1 = twist_to_k1.pantilt_to_k1:main',
            'odom_to_tf = twist_to_k1.odom_to_tf:main'
        ],
    },
)
