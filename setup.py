from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kompass_sim'

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*.*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "maps"),
            glob(os.path.join("maps", "*.*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Automatika Robotics",
    maintainer_email="contact@automatikarobotics.com",
    description="Kompass Simulations and Tests",
    license="MIT License Copyright (c) 2024 Automatika Robotics",
)
