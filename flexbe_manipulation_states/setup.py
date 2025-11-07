"""
Setup for flexbe_manipulation_states
"""

from glob import glob

from setuptools import setup
from setuptools import find_packages

PACKAGE_NAME = 'flexbe_manipulation_states'

setup(
    name=PACKAGE_NAME,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + "/tests", glob('tests/*.test')),
        ('share/' + PACKAGE_NAME + "/launch", glob('tests/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    author='phil',
    author_email='philsplus@gmail.com',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_manipulation_states provides a collection of robot-agnostic states'
                ' related to manipulation and trajectory execution.',
    license='BSD',
    entry_points={
        'console_scripts': [
        ],
    },
)
