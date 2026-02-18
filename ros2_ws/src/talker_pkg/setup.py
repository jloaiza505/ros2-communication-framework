from setuptools import find_packages, setup

package_name = 'talker_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='jloaiza505@gmail.com',
    description='ROS2 publisher node for periodic chatter topic output.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'talker_node = talker_pkg.talker_node:main',
        ],
    },
)
