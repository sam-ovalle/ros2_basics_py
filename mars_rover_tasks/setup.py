from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mars_rover_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_obstacle_detector_executable = mars_rover_tasks.subscriber_obstacle_detector:main',
            'publish_mars_rover_move_executable = mars_rover_tasks.publish_mars_rover_move:main',
            'image_plant_detector_executable = mars_rover_tasks.plant_detector_node:main',
            'autonomous_exploration_executable = mars_rover_tasks.autonomous_exploration:main', 
            'text_recog_executable = mars_rover_tasks.text_recog_node:main',
        ],
    },
)
