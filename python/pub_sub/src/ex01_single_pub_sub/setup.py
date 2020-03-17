from setuptools import find_packages
from setuptools import setup

setup(
    name='ex01_single_pub_sub',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools'],
    author='Raffaele Grandi',
    author_email='raffaele.grandi@gmail.com',
    maintainer='RaffaeleGrandi',
    maintainer_email='raffaele.grandi@gmail.com',
    keywords=['ROS2'],
    description= 'Basic example with a simple sender and a simple receiver',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sender = basic_sender.basic_sender:main',
            'receiver = basic_receiver.basic_receiver:main',
        ],
    },
)
