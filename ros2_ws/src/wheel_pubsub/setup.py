from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wheel_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='la-user',
    maintainer_email='giacomoburani11@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_buffer = wheel_pubsub.wheel_buffer:main',
            'stepper_ctrl = wheel_pubsub.stepper_control:main',
            'listener = wheel_pubsub.wheel_subscriber:main',
            'control = wheel_pubsub.control_pub:main',

        ],
    },
)
