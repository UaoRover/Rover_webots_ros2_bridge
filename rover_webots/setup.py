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
    'launch/webots.launch.py'
]))
data_files.append(('share/' + package_name, [
    'launch/rviz_webots_real.launch.py'
]))
data_files.append(('share/' + package_name, [
    'launch/obstaculos_rviz_webots.launch.py'
]))
data_files.append(('share/' + package_name, [
    'launch/simulation_localization.launch.py'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/entorno_real.wbt'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/entorno_obstaculos.wbt'
]))
data_files.append(('share/' + package_name + '/worlds/meshes', [
    'worlds/meshes/Terrain.stl'
]))
data_files.append(('share/' + package_name + '/worlds/meshes', [
    'worlds/meshes/rock.obj'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_1.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_2.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_3.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_4.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_5.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_6.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_7.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_8.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_9.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_10.jpeg'
]))
data_files.append(('share/' + package_name + '/worlds/pictures', [
    'worlds/pictures/Poste_11.jpeg'
]))
data_files.append(('share/' + package_name + '/params', [
    'params/dual_ekf_navsat_example.yaml'
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
            'teleop=rover_webots.teleop:main',
            'main=rover_webots.main:main',
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
