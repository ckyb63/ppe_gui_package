from setuptools import setup
import os
from glob import glob

package_name = 'gui_package'

setup(
    name=package_name,
    version='0.3.1',
    packages=[package_name],
    data_files=[
        # Explicitly specify the marker file installation
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add any launch files if you have them
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max Chen',
    maintainer_email='ckyb63@gmail.com',
    description='GUI package for PPE vending machine',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ppe_gui = gui_package.ppe_gui:main',
            'experimental_gui = gui_package.experimental_ppe_gui:main',
            'dummy_ppe = gui_package.dummy_ppe_status:main',
        ],
    },
)
