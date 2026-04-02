from setuptools import find_packages, setup

package_name = 'text2speech'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        'text2speech': ['models/*'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocup',
    maintainer_email='garcia.miguel.onate@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pipertts = text2speech.pipertts:main'
        ],
    },
)
