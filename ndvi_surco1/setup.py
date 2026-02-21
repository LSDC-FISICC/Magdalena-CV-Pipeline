from setuptools import setup
import glob
import os

package_name = 'ndvi_surco1'

def _package_files(glob_pattern):
    return [p for p in glob.glob(glob_pattern)]

setup(
    name=package_name,
    version='0.0.0',
    packages=['ndvi_surco1'],
    data_files=[
        ('share/ament_index/resource_index/ament_python', ['resource/ndvi_surco1']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', _package_files('launch/*.py')),
        ('share/' + package_name + '/config', _package_files('*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple ROS2 Python package',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nocheyndvi = ndvi_surco1.nocheyndvi:main',
            'camerachek = ndvi_surco1.cameracheck:main',
        ],
    },
)

