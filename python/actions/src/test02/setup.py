from setuptools import find_packages
from setuptools import setup

setup(
    name='mini_action_single_node_threads',
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
    description= 'Basic example of minimal action client and server threads communicating on single node',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_test02 = mini_action_single_node_threads.action_mini:main'
        ],
    },
)
