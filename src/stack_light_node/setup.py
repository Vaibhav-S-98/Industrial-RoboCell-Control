from setuptools import setup

package_name = 'stack_light_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vsaini',
    maintainer_email='vsaini@example.com',
    description='stack light node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_light_node = stack_light_node.stack_light_node:main',
        ],
    },
)
