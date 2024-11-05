from setuptools import find_packages, setup

package_name = 'P7'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaulab',
    maintainer_email='aaulab@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'JohnsFunciton = image_subscriber.image_subscriber:main'
=======
<<<<<<< HEAD
            'JohnsFunciton = image_subscriber.image_subscriber:main',
            'yolo11_subscriber = yolo11_subscriber.yolo11_subscriber:main'
=======
            'JohnsFunciton = image_subscriber.image_subscriber:main', 
            'yolo11_subscriber = yolo11_subscriber.yolo11_subscriber:main'
>>>>>>> origin/fine_tune_hand_guestures
>>>>>>> db6c6324138a7ba00265d32bf682105d983d09ab
        ],
    },
)
