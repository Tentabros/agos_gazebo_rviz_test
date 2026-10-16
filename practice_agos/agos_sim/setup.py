from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'agos_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl') + glob('meshes/*.obj') + glob('meshes/*.mtl')),
        (os.path.join('share', package_name, 'materials', 'textures'),
            glob('materials/textures/*')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tentabros',
    maintainer_email='reuel.lichangco@gmail.com',
    description='AGOS sewer inspection robot simulation: Gazebo worlds, RTAB-Map SLAM, '
                'Nav2 and TAD frontier exploration',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'explorer = agos_sim.explorer:main',
            'nav_supervisor = agos_sim.nav_supervisor:main',
            'odom_gps = agos_sim.odom_gps:main',
            'imu_level = agos_sim.imu_level:main',
        ],
    },
)
