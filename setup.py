from setuptools import setup

package_name = 'gui_package'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max Chen',
    maintainer_email='ckyb63@gmail.com',
    description='PyQt5-based GUI for PPE Vending Machine with ROS2 integration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ppe_gui = gui_package.ppe_gui:main',
            'dummy_ppe = gui_package.dummy_ppe_status:main',
        ],
    },
)
