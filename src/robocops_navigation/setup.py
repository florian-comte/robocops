from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robocops_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'robocops_navigation'), glob('robocops_navigation/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='micdev',
    maintainer_email='michel.abela@epfl.ch',
    description='This package aims at running SLAM toolbox and Nav2 for RoboCops',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_filter = robocops_navigation.filter_lidar_data:main'
        ],
    },
)
