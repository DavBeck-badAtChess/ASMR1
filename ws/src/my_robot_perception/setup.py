from setuptools import find_packages, setup

package_name = 'my_robot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_fusion_node = my_robot_perception.scan_fusion_node:main',
        ],
    },
)
