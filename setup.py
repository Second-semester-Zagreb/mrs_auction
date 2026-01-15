from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mrs_auction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools', 'networkx'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='baothif4ntp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_executor_node = mrs_auction.mission_executor:main',
            'simple_executor_node = mrs_auction.simple_executor:main',
        ],
    },
    scripts=[
        'scripts/tasks_markers.py',
    ],
)
