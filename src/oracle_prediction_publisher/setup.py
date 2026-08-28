from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'oracle_prediction_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='w417',
    maintainer_email='w417@example.com',
    description='Deterministic Oracle future occupancy publisher for Gate 3.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'oracle_prediction_publisher = '
            'oracle_prediction_publisher.publisher:main',
        ],
    },
)
