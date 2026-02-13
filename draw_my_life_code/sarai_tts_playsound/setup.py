import os
from glob import glob
from setuptools import setup

package_name = 'sarai_tts_playsound'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'voice_model'), glob('voice_model/*.onnx')),
        (os.path.join('share', package_name, 'voice_model'), glob('voice_model/*.onnx.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maure',
    maintainer_email='romain.maure@kit.edu',
    description='Package allowing any robot to speak.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sarai_tts_playsound_node = sarai_tts_playsound.sarai_tts_playsound_node:main'
        ],
    },
)
