from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ur5_moveit_scripts'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuenze',
    maintainer_email='tuenze3@gmail.com',
    description='Simulation-only UR5 MoveIt 2 examples using mock ros2_control hardware.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur5_joint_goal = ur5_moveit_scripts.ur5_joint_goal:main',
            'ur5_pose_goal = ur5_moveit_scripts.ur5_pose_goal:main',
            'ur5_go_home = ur5_moveit_scripts.ur5_go_home:main',
            'ur5_sim_demo = ur5_moveit_scripts.ur5_sim_demo:main',
            'ur5_pick_place_demo = ur5_moveit_scripts.ur5_pick_place_demo:main',
        ],
    },
)
