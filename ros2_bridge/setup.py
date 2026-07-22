from setuptools import setup

package_name = 'rzv2h_detection_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jian-You Chen',
    maintainer_email='nfuee0102@gmail.com',
    description=(
        "Republishes the RZ/V2H camera pipeline's UDP-JSON detection "
        'sidecar as standard ROS 2 vision_msgs topics.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = rzv2h_detection_bridge.bridge_node:main',
        ],
    },
)
