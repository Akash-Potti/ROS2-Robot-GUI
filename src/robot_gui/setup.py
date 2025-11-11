from setuptools import setup

package_name = 'robot_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name, 
        f'{package_name}.gui', 
        f'{package_name}.gui.widgets',
        f'{package_name}.ros_interface', 
        f'{package_name}.data_logger'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akash',
    maintainer_email='akashpotti07@gmail.com',
    description='A ROS2 GUI that visualizes and controls a simulated robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_gui = robot_gui.main:main'
        ],
    },
)
