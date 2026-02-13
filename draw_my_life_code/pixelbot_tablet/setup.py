import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pixelbot_tablet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/saved_drawings', []),
        (os.path.join('share', package_name, 'images'), glob('images/*.png'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maure',
    maintainer_email='romain.maure@kit.edu',
    description='Drawing application for the Draw My Life activity',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "draw_node = pixelbot_tablet.drawing_app_node:main"
        ],
    },
)
