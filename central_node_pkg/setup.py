from setuptools import setup

package_name = 'central_node_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/central_node.launch.py']),
        ('share/' + package_name + '/config', ['config/central_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin30',
    maintainer_email='orin30@example.com',
    description='Nodo central que se suscribe a varios tópicos y publica cada 10 ms.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'central_node = central_node_pkg.central_node:main',
        ],
    },
)

