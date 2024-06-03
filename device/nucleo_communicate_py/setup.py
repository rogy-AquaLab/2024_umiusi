import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'nucleo_communicate_py'

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
    description='Nucleoとの通信周り',
    license='MIT',
    tests_require=[],
    entry_points={
        'console_scripts': [
            "sender = nucleo_communicate_py.sender:main",
            "receiver = nucleo_communicate_py.receiver:main",
            "channel = nucleo_communicate_py.channel:main"
        ],
    },
)
