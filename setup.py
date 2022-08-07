from setuptools import setup

package_name = 'transportino_lidar'

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
    maintainer='mattsays',
    maintainer_email='mattsays@transportino.tech',
    description='Driver for lidar module mb_1r2t',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = transportino_lidar.lidar_node:main'
        ],
    },
)
