from setuptools import find_packages, setup

package_name = 'camera_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiro',
    maintainer_email='yoshikawa.h.ah@m.titech.ac.jp',
    description='Camera for ROS2',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera=camera_reader.camera:main',
        ],
    },
)