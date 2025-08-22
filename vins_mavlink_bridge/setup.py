from setuptools import setup

package_name = 'vins_mavlink_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pymavlink','numpy'],
    zip_safe=True,
    maintainer='Christopher',
    maintainer_email='chrissutandar@gmail.com',
    description='Bridge VINS Odometry to MAVLink ODOMETRY',
    entry_points={
        'console_scripts': [
            'bridge = vins_mavlink_bridge.bridge_node:main',
        ],
    },
)
