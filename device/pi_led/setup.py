import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pi_led"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="23-saho",
    maintainer_email="sahonorth@gmail.com",
    description="LEDを光らせる",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pi_led = pi_led.pi_led:main",
        ],
    },
)
