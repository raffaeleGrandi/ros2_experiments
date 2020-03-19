from setuptools import find_packages
from setuptools import setup

setup(
    name='test01',
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
    description= 'Template example for tests',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = main_program.main_file:main',
        ],
    },
)
