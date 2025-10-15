from setuptools import setup

package_name = 'turtlebot3_emergency'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/launch', ['launch/emergency_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team6',
    maintainer_email='team6@example.com',
    description='Emergency stop controller toggling TurtleBot3 motor power.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_bridge = turtlebot3_emergency.emergency_bridge:main',
        ],
    },
)
