import os
from glob import glob
from setuptools import setup

package_name = 'pixelbot_display'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'imgs'), glob('imgs/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maure',
    maintainer_email='romain.maure@kit.edu',
    description='Pygame face animation on LCD display',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixelbot_display_node = pixelbot_display.pixelbot_display_node:main'
        ],
    },
)
