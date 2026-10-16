from setuptools import setup

package_name = 'rto_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahul',
    maintainer_email='karahul209@gmail.com',
    description='Keyboard teleoperation of Robotino and the safety layer of teleoperation',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'keyboard_teleop_tty = rto_teleop.keyboard_teleop_tty:main',
            'teleop_safety = rto_teleop.teleop_safety:main',
        ],
    },
)
