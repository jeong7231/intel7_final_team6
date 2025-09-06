from setuptools import setup

package_name = 'aruco_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='ArUco marker detection and pose estimation with TF',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_pose_estimation.aruco_marker_detector:main',
            'aruco_pose_tf = aruco_pose_estimation.aruco_marker_pose_estimation_tf:main',
        ],
    },
)

