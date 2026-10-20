from setuptools import setup

package_name = 'rto_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ... Other data files
        # Include all launch files.
        ('share/' + package_name, ['launch/rto_bringup_launch.py']),
        ('share/' + package_name, ['launch/rto_rviz_launch.py']),
        ('share/' + package_name, ['launch/joy_teleop_launch.py']),
        ('share/' + package_name, ['launch/slam_launch.py']),
        ('share/' + package_name, ['launch/amcl_launch.py']),
        ('share/' + package_name, ['launch/teleop_safety_launch.py']),
        ('share/' + package_name, ['launch/gripper_launch.py']),
        ('share/' + package_name + '/config', [
            'config/amcl.yaml',
            'config/gripper.yaml',
            'config/joy_teleop.yaml',
            'config/slam_toolbox_localization.yaml',
            'config/slam_toolbox_mapping.yaml',
            'config/teleop_safety.yaml',
        ]),
        ('share/' + package_name, [
            'rviz/amcl.rviz',
            'rviz/rto_config.rviz',
            'rviz/slam.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahul',
    maintainer_email='karahul209@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
        ],
    },
)
