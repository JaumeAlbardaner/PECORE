import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'tf_adder'

setup(
    name=package_name,
    version='0.0.5',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaume',
    maintainer_email='jaal5534@colorado.edu',
    description='Package that publishes the map frame for a given Jackal robot running in Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_adder = tf_adder.map_adder:main'
        ],
    },
)
