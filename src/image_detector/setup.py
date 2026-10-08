from setuptools import find_packages, setup

package_name = 'image_detector'

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
    description='Detects objects with YOLO in the Ouster near-IR image (or any image topic).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_detector_node = image_detector.image_detector_node:main'
        ],
    },
)
