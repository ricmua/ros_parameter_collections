from setuptools import setup

package_name = 'ros_parameter_collections'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a.whit',
    maintainer_email='ros@whit.contact',
    description='Wrappers for ROS2 nodes that provide an interface for treating ROS2 parameter groups like Python collection data types.',
    license='Mozilla Public License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
