from setuptools import find_packages, setup

package_name = 'my_robot_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'goal_checker_node = my_robot_perception.goal_checker_node:main',
            'scan_to_pointcloud_node = my_robot_perception.scan_to_pointcloud_node:main',
        ],
    },
)
