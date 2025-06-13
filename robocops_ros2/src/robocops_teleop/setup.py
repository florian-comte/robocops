from setuptools import find_packages, setup

package_name = 'robocops_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florian Comte',
    maintainer_email='florian.comte@epfl.ch',
    description='Package used by the team RocoCops (Robot Competition EPFL 2025) to be able to control in gazebo and in real life the robot using teleoperator keyboard',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = robocops_teleop.robocops_teleop_keyboard:main'
        ],
    },
)
