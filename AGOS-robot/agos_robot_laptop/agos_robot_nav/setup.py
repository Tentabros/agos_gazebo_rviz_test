from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'agos_robot_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tentabros',
    maintainer_email='reuel.lichangco@gmail.com',
    description="The AGOS test robot's laptop side: Nav2, RViz and frontier exploration",
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'explorer = agos_robot_nav.explorer:main',
            'nav_supervisor = agos_robot_nav.nav_supervisor:main',
        ],
    },
)
