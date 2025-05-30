from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_challenge_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Incluir archivos de launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Incluir archivo URDF
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Incluir archivos STL
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        # Incluir config. RViz2
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='A01736171@tec.mx',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation = mini_challenge_5.localisation:main',
            'rs_robot = mini_challenge_5.real_sim_robot:main',
            'control = mini_challenge_5.control_node:main',
            'set_point = mini_challenge_5.set_point_generator:main',
            'path_publisher = mini_challenge_5.path_publisher:main',
            'ekf = mini_challenge_5.ekf:main',
            'aruco_detection = mini_challenge_5.aruco_detection:main',
        ],
    },
)
