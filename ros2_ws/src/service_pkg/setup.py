from glob import glob
from setuptools import find_packages, setup

package_name = 'service_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='jloaiza505@gmail.com',
    description='ROS2 SetBool service package for enabling and disabling the talker.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'service_server = service_pkg.service_server:main',
            'service_client = service_pkg.service_client:main',
        ],
    },
)
