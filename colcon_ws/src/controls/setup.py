import os
from glob import glob
from setuptools import setup

package_name = "controls"

setup(
    name=package_name,
    version='0.1.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='McGill Robotics',
    author_email='dev@mcgillrobotics.com',
    maintainer='McGill Robotics',
    maintainer_email='dev@mcgillrobotics.co',
    keywords=['controls', 'McGill'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: GPLv3',
        'Programming Language :: Python',
        'Topic :: Robotics Development',
    ],
    description='Controls Package for McGill Robotics AUV Subteam',
    license='GPLv3',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            
        ],
    },
)