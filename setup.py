
import os
from glob import glob
from setuptools import setup

package_name = 'rqt_joint_trajectory_plot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.ui')),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
        # (os.path.join(os.path.join('share', package_name), "config"), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='George Dwyer',
    maintainer_email='george.dwyer@ucl.ac.uk',
    description='RQT components for TF',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

