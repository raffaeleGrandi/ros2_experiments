#!/usr/bin/env python

import argparse
import sys
import os


def create_main_file(base_dir, main_file, main_function, dependencies):

    print("Creating main file...")

    file_path = base_dir+'/'+package_name+'/'+main_file+'.py'
    file = open(file_path,'w')

    for i_dep in dependencies:
        file.write("import "+i_dep+"\n")

    file.write("from rclpy.node import Node\n")
    file.write("from std_msgs.msg import String\n")

    for i in range(5):
        file.write("\n")

    file.write("def "+main_function+"(args=None):\n")
    file.write("    rclpy.init(args=args)\n")
    for i in range(3):
        file.write("    ... \n")
    file.write("    rclpy.shutdown()\n")
    file.write("\n")
    file.write("\n")
    file.write("if __name__ == '__main__':\n")
    file.write("    "+main_function+"()")

    file.flush()
    file.close()


def create_setup_py(base_dir, author_name, author_email, package_name, package_version,
    license, script_name, main_file, main_function):

    print("Creating setup.py...")

    file_path = base_dir+'/'+'setup.py'
    file = open(file_path,'w')

    file.write("from setuptools import find_packages\n")
    file.write("from setuptools import setup\n\n")

    file.write("setup(\n")
    file.write("    name='"+package_name+"',\n")
    file.write("    version='"+package_version+"',\n")
    file.write("    packages=find_packages(exclude=['test']),\n")
    file.write("    py_modules=[],\n")
    file.write("    zip_safe=True,\n")
    file.write("    install_requires=['setuptools'],\n")
    file.write("    author='"+author_name+"',\n")
    file.write("    author_email='"+author_email+"',\n")
    file.write("    maintainer='AnteMotion',\n")
    file.write("    maintainer_email='"+author_email+"',\n")
    file.write("    keywords=['ROS2'],\n")
    file.write("    description='"+package_name+" package: TODO',\n")
    file.write("    license='"+license+"',\n")
    file.write("    tests_require=['pytest'],\n")
    file.write("    entry_points={\n")
    file.write("        'console_scripts': [\n")
    file.write("            '"+script_name+" = "+package_name+"."+main_file+":"+main_function+"'\n")
    file.write("        ],\n")
    file.write("    },\n")
    file.write(")\n")

    file.flush()
    file.close()


def create_setup_cfg(base_dir, package_name):
    
    print("Creating setup.cfg...")

    file_path = base_dir+'/'+'setup.cfg'
    file = open(file_path,'w')
    file.write('[develop]\n')
    file.write('script-dir=$base/lib/'+package_name+'\n')
    file.write('[install]\n')
    file.write('install-scripts=$base/lib/'+package_name+'\n')
    file.flush()
    file.close()


def create_package_xml(base_dir,author_email,package_name,package_version,license,dependencies):

    print("Creating package.xml...")

    file_path = base_dir+'/'+'package.xml'
    file_package_xml = open(file_path,'w')

    file_package_xml.write('<?xml version="1.0"?>\n')
    file_package_xml.write('<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>\n')
    file_package_xml.write('<package format="2">\n')
    file_package_xml.write('    <name>'+package_name+'</name>\n')
    file_package_xml.write('    <version>'+package_version+'</version>\n')
    file_package_xml.write('    <description>'+package_name+': TODO </description>\n')
    file_package_xml.write('\n')
    file_package_xml.write('    <maintainer email="'+author_email+'">AnteMotion</maintainer>\n')
    file_package_xml.write('    <license>'+license+'</license>\n')
    file_package_xml.write('\n')

    for i_dep in dependencies:
        file_package_xml.write('    <exec_depend>'+i_dep+'</exec_depend>\n')

    file_package_xml.write('\n')
    #file_package_xml.write('    <!-- These test dependencies are optional\n')
    #file_package_xml.write('    Their purpose is to make sure that the code passes the linters -->\n')
    #file_package_xml.write('    <test_depend>ament_copyright</test_depend>\n')
    #file_package_xml.write('    <test_depend>ament_flake8</test_depend>\n')
    #file_package_xml.write('    <test_depend>ament_pep257</test_depend>\n')
    #file_package_xml.write('    <test_depend>python3-pytest</test_depend>\n')
    #file_package_xml.write('\n')
    file_package_xml.write('    <export>\n')
    file_package_xml.write('        <build_type>ament_python</build_type>\n')
    file_package_xml.write('    </export>\n')
    file_package_xml.write('</package>\n')

    file_package_xml.flush()
    file_package_xml.close()


def init_structure(base_dir, author_email, package_name, package_version, license,
    dependencies, script_name, main_file, main_function):

    print("Executing init structure....")

    if not os.path.exists(base_dir):
        os.makedirs(base_dir+'/'+package_name)
        os.mknod(base_dir+'/'+package_name+'/'+'__init__.py')

        dependencies += ['rclpy','std_msgs']
        dependencies = list(set(dependencies))
        dependencies.sort()

        create_main_file(base_dir, main_file, main_function, dependencies)
        create_setup_py(base_dir,author_name, author_email, package_name, package_version, license,
            script_name, main_file, main_function)
        create_setup_cfg(base_dir, package_name)
        create_package_xml(base_dir,author_email,package_name,package_version,license,dependencies)
    else:
        print('ERROR: The directory of the package exists. Delete it before init package structure')


if __name__=='__main__':

    author_name = 'Raffaele Grandi'
    author_email = 'raffaele.grandi@antemotion.com'
    package_version = '0.0.1'
    license = 'Proprietary'
    main_function = 'main'

    '''
    parser = argparse.ArgumentParser(description="ROSPy structure builder")
    parser.add_argument("-pn ","--package-name", dest='package_name', required=True, type=str, help="Name of the main package")
    parser.add_argument("-en ","--executable-name", dest='executable_name', required=True, type=str, help="Name of the executable used in 'ros2 run <executable_name>'")
    parser.add_argument("-mf ","--main-file", dest='main_file', required=True, type=str, help="Name of the file contaning the main procedure")
    parser.add_argument("-d ", "--dependencies", dest='dependencies', default="", required=False, type=str, help="Dependencies of the package 'dep1 dep2 ... depN'")
    args = parser.parse_args()

    base_dir = 'src/'+args.package_name
    package_name = args.package_name
    executable_name = args.executable_name
    main_file = args.main_file
    dependencies = args.dependencies.split()
    '''

    package_name = 'my_package'
    base_dir = 'src/'+ package_name
    executable_name = 'executable'
    main_file = 'main_file'
    dependencies = [] 
    
    init_structure(base_dir, author_email, package_name, package_version, license,
        dependencies, executable_name, main_file, main_function)
