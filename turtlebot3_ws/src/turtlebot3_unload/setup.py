from setuptools import setup

package_name = 'turtlebot3_unload'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/destinations.yaml']),
        ('share/' + package_name + '/launch', ['launch/unload.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team6',
    maintainer_email='team6@example.com',
    description='TurtleBot3 unload mission',
    license='Apache-2.0',
    entry_points={'console_scripts': ['unload_node = turtlebot3_unload.unload_node:main']},
)
