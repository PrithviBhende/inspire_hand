from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inspire_hand'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'meshes', 'visual'), glob('meshes/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'collision'), glob('meshes/collision/*')),
        (os.path.join('share',package_name),glob('config/*')),
    ],
    install_requires=['setuptools',
    'rclpy',  # For ROS2 Python API
    'sensor_msgs',
    'trajectory_msgs',
    ],
    zip_safe=True,
    maintainer='psb',
    maintainer_email='psb@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'move_finger = inspire_hand.move_finger:main',
            'joint_angle_monitor = inspire_hand.joint_angle_monitor:main',
            'joint_initializer = inspire_hand.joint_initializer:main',
            'move_fingers = inspire_hand.move_fingers:main',
        ],
    },
)
