from setuptools import setup

package_name = 'turtlebot3_unload'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/config', ['config/destinations.yaml']),
        (f'share/{package_name}/launch', [
            'launch/unload_mission.launch.py',
            'launch/unload_system.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='team6',
    maintainer_email='team6@example.com',
    description='Mission manager for TurtleBot3 automatic unloading sequence.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = turtlebot3_unload.mission_manager:main',
        ],
    },
)
