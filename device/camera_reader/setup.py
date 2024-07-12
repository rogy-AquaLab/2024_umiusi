import os
from glob import glob

from setuptools import setup

package_name = "camera_reader"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hiro",
    maintainer_email="yoshikawa.h.ah@m.titech.ac.jp",
    description="Camera for ROS2",
    license="MIT license",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "camera=camera_reader.camera:main",
        ],
    },
)
