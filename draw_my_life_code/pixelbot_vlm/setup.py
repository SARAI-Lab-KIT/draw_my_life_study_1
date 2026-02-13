import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pixelbot_vlm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'vlm_persona'), glob('vlm_persona/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maure',
    maintainer_email='romain.maure@kit.edu',
    description='Analyze children drawings and verbal input in the context of the draw my life activity',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixelbot_vlm_node = pixelbot_vlm.pixelbot_vlm_node:main'
        ],
    },
)
