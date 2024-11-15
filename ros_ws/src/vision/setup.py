import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaulab',
    maintainer_email='jonas.thorhauge@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = vision.image_subscriber:main', 
            'yolo11_subscriber = vision.yolo11_subscriber:main',
            'rgb_subscriber = vision.rgb_subscriber:main',
            'depth_subscriber = vision.depth_subscriber:main',
            'action_decision = vision.action_decision:main', 
            'gesture_handler = vision.gesture_handler:main'
        ],
    },
)
