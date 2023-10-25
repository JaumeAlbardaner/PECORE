from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'first_deliverable'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_remap = first_deliverable.tf_remap:main',
            'pbvs = first_deliverable.pbvs:main'
        ],
    },
)
