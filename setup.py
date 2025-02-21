from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'gui_package'

setup(
    name=package_name,
    version='0.8.3',
    packages=find_packages(),
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
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_ppe_gui = gui_package.main_gui_modules.main:main',
            'dummy_ppe = gui_package.dummy_test.dummy_ppe_status:main',
            'dummy_inventory = gui_package.dummy_test.dummy_inventory_publisher:main',
            'safety_gate_controller = gui_package.gate.safety_gate_controller:main',  
            'record_dispense_bag = gui_package.main_gui_modules.recordedBags.record_dispense_bag:main',  
        ],
    },
)
