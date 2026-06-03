from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'semantic_object_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rahiitya',
    maintainer_email='rahiitya@todo.todo',
    description='ROS2 Humble RGB-D semantic segmentation and legacy object detection.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgbd_semantic_segmenter = semantic_object_detection.rgbd_semantic_segmenter:main',
            'yolo_detector = semantic_object_detection.yolo_detector:main',
            'color_detector = semantic_object_detection.color_detector:main',
            'combined_detector = semantic_object_detection.combined_detector:main',
        ],
    },
)
