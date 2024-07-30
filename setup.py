import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bag_to_csv'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalman',
    maintainer_email='samuele.sandrini@polito.it',
    description='Bag to csv generator',
    license='Apache License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_to_csv_node = bag_to_csv.bag_to_csv:main'
    ],
    },
)
