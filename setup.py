from setuptools import find_packages, setup

package_name = 'fertilizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_nodes.py']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml', 'pyModbusTCP'],
    zip_safe=True,
    maintainer='LSDC',
    maintainer_email='lsdc@galileo.edu',
    description='Package for fertilizer application control using ROS2. This project uses several D555 cameras to detect presence or absence of plants, and block flow of fertilizer accordingly.',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'processing_pipeline = fertilizer.processing_pipeline:main',
            'ndvi_threshold = fertilizer.ndvi_threshold:main',
            'modbus_controller = fertilizer.modbus_controller:main',
        ],
    },
)
