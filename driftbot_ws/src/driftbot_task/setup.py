from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'driftbot_task'

share = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(share, 'launch'), glob('launch/*.py')),
        (os.path.join(share, 'config'), glob('config/*.yaml')),
        (os.path.join(share, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='ujwalwag',
    maintainer_email='ujwalwag@todo.todo',
    description='Scripted Gazebo mission: bridges, TF/joint/scan support, pick/drop.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_node = driftbot_task.task_node:main',
            'gz_bridge_support = driftbot_task.gz_bridge_support:main',
        ],
    },
)
