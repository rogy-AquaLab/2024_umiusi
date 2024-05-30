import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'pi_i2c'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='h1rono',
    maintainer_email='hronok66@gmail.com',
    description='Raspberry PiのI2C接続',
    license='MIT',
    tests_require=[],
    entry_points={
        'console_scripts': [
            "depth = pi_i2c.depth:main",
            "imu = pi_i2c.imu:main",
            "all = pi_i2c.all:main"
        ],
    },
)
