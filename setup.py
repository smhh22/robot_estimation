from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smhh22',
    maintainer_email='s.m.m.m.h2013@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'low_pass_filter = robot_estimation.low_pass_filter:main',
            'bias_calculator = robot_estimation.bias_calculator:main',
            'controller_motion_model = robot_estimation.controller_motion_model:main',
            'odom_estimator = robot_estimation.odom_estimator:main',
        ],
    },
)
