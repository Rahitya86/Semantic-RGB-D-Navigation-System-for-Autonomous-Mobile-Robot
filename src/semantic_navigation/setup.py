from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'semantic_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rahiitya',
    maintainer_email='rahiitya@todo.todo',
    description='RGB-D semantic navigation tools for K12 Gazebo, Nav2, and RTAB-Map.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_cloud_generator = semantic_navigation.semantic_cloud_generator:main',
            'gazebo_semantic_landmarks = semantic_navigation.gazebo_semantic_landmarks:main',
            'semantic_goal_resolver = semantic_navigation.semantic_goal_resolver:main',
            'nav2_goal_bridge = semantic_navigation.nav2_goal_bridge:main',
            'clean_2d_map = semantic_navigation.clean_2d_map:main',
            'full_world_mapper = semantic_navigation.full_world_mapper:main',
            'cmd_vel_limiter = semantic_navigation.cmd_vel_limiter:main',
        ],
    },
)
