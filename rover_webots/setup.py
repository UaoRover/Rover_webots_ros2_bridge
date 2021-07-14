import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'rover_webots'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))
data_files.append(('share/' + package_name, [
    'launch/demo_launch.py'
]))
data_files.append(('share/' + package_name, [
    'launch/rover_launch.py'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/Prueba_rover_1_sin_rocker.wbt'
]))
data_files.append(('share/' + package_name, [
    'launch/rectangle_launch.py'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/Prueba de Visual.wbt'
]))
data_files.append(('share/' + package_name + '/worlds/meshes', [
    'worlds/meshes/Terrain.stl'
]))

data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/ament_index/resource_index/packages',['resource/' + package_name]))
setup(    
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools','launch'],
    zip_safe=True,
    maintainer='alejoxbg',
    maintainer_email='alejandro.naranjo_z@uao.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_vel = rover_webots.publisher_vel:main',
            'master=rover_webots.master:main',
            'slave=rover_webots.slave:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
