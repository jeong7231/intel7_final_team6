from setuptools import setup

package_name = 'turtlebot3_auto_unload'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/destinations.example.yaml', 'config/destinations.yaml']),
        (f'share/{package_name}/launch', ['launch/auto_unload.launch.py', 'launch/auto_unload_system.launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='team6',
    maintainer_email='team6@example.com',
    description='Mission coordination node for TurtleBot3 automatic unloading workflow.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = turtlebot3_auto_unload.mission_manager:main',
        ],
    },
)
