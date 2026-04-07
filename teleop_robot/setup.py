from setuptools import find_packages, setup

package_name = 'teleop_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jothi Venkatesh K',
    maintainer_email='jothivenkateshk@gmail.com',
    description='Teleoperation of Robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_key = teleop_robot.teleop_key:main',
        ],
    },
)
