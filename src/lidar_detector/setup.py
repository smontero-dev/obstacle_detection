from setuptools import find_packages, setup

package_name = 'lidar_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santi',
    maintainer_email='santiago.montero@alumnos.upm.es',
    description='Detects obstacles in the Ouster point cloud (ground removal and clustering).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_detector_node = lidar_detector.lidar_detector_node:main'
        ],
    },
)
