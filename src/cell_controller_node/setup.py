from setuptools import setup

package_name = 'cell_controller_node'

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
    maintainer_email='your@email.com',
    description='cell controller node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cell_controller_node = cell_controller_node.cell_controller_node:main',
        ],
    },
)
