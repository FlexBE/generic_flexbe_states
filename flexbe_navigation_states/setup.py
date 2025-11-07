from setuptools import setup
from setuptools import find_packages

package_name = 'flexbe_navigation_states'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    author='phil',
    author_email='philsplus@gmail.com',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_navigation_states provides a collection of robot-agnostic '
                'states related to mobile robot navigation.',
    license='BSD',
    entry_points={
        'console_scripts': [
        ],
    },
)
