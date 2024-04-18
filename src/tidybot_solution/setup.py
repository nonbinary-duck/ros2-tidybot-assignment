import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tidybot_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # From https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Also include our custom params files
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='villanelle',
    maintainer_email='46252182+nonbinary-duck@users.noreply.github.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'identify_cubes = tidybot_solution.identify_cubes:main',
            'tidy_cubes = tidybot_solution.tidy_cubes:main'
        ],
    },
)
