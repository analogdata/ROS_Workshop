from setuptools import find_packages, setup

package_name = 'keyboard_node'

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
    maintainer='rajath',
    maintainer_email='analogdata.io@gmail.com',
    description='ROS2 keyboard-controlled LED via UDP to ESP8266 (3 nodes)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_input_node = keyboard_node.keyboard_input_node:main',
            'udp_sender_node = keyboard_node.udp_sender_node:main',
            'status_display_node = keyboard_node.status_display_node:main',
        ],
    },
)
