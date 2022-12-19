#!/usr/bin/env python3

import os

from glob import glob
from setuptools import setup

package_name = "canon"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.csv")),
        (os.path.join("share", package_name, "data"), glob("data/*"))],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Varundev Sukhil",
    maintainer_email="vsukhil@gmail.com",
    description="Canon: A collection of code and documents to describe my journey through graduate school",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"{node} = {package_name}.{node}:{node}" for node in [
                "bounds_logger", 
                "path_tracker", 
                "path_server",
                "argos",
                "oracle"
            ]
        ],
    },
)
