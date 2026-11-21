import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pumasim_booster_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LIRA-UNAM Pumanoids",
    maintainer_email="pumanoids@lira.unam.mx",
    description="Booster RoboCup demo protocol and localization bridge over PumaSim generic ROS 2 topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "booster_bridge = pumasim_booster_bridge.bridge_node:main",
        ],
    },
)
