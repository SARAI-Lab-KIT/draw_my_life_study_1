from setuptools import find_packages, setup

package_name = 'sarai_speech_recognition'

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
    maintainer='maure',
    maintainer_email='romain.maure@kit.edu',
    description='Package allowing any robot with a microphone to recognize speech',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sarai_speech_recognition_node = sarai_speech_recognition.sarai_speech_recognition_node:main'
        ],
    },
)
