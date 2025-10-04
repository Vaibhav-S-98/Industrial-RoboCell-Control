from setuptools import setup

package_name = 'scanner_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vsaini',
    maintainer_email='vsaini@example.com',
    description='Scanner node for ROS2 bin picking project',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scanner_node = scanner_node.scanner_node:main',
        ],
    },
)
