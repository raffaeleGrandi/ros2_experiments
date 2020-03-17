from setuptools import find_packages
from setuptools import setup

setup(
    name='ex02_multi_pub_sub',
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
    description= 'Basic example with multiple senders and multiple receivers with round trip timing',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sender = multi_sender.multi_sender:main',
            'receiver = multi_receiver.multi_receiver:main',
        ],
    },
)
