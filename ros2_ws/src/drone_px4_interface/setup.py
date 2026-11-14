from setuptools import find_packages, setup

package_name = 'drone_px4_interface'

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
    maintainer='Sumedh Peramanu',
    maintainer_email='sumedh.peramanu@gmail.com',
    description='Launch files and config that start the drone software stack',
    license='APACHE-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_reader = drone_px4_interface.odom_reader:main',
            'offboard_takeoff = drone_px4_interface.offboard_takeoff:main'
        ],
    },
)
