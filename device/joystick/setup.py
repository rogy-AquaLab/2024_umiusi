from setuptools import find_packages, setup

package_name = 'joystick'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='h1rono',
    maintainer_email='hronok66@gmail.com',
    description='ゲームコントローラー',
    license='MIT',
    tests_require=[],
    entry_points={
        'console_scripts': [
            "joystick = joystick.joystick:main"
        ],
    },
)
