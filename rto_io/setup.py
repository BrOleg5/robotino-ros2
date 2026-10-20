from setuptools import setup

package_name = 'rto_io'

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
    maintainer='BrOleg5',
    maintainer_email='oleg.brylyov@mail.ru',
    description='Gripper and status LEDs connected to the I/O interface of Robotino',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gripper = rto_io.gripper:main',
        ],
    },
)
