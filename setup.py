from setuptools import find_packages, setup
from glob import glob

package_name = 'examen_4_2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gnuno',
    maintainer_email='gnuno@todo.todo',
    description='Motor controller velocity and position via ESP32 micro-ROS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Controllers
            'pid_controller      = examen_4_2.pid_controller:main',
            'smc_controller      = examen_4_2.smc_controller:main',
            'ismc_controller     = examen_4_2.ismc_controller:main',
            'ctc_controller      = examen_4_2.ctc_controller:main',
            'ph_controller       = examen_4_2.ph_controller:main',
            # Teleop
            'teleop_keyboard     = examen_4_2.teleop_keyboard:main',
            # Dashboard
            'dashboard           = examen_4_2.dashboard:main',
            # Terrain
            'terrain_perturb     = examen_4_2.terrain_perturbation:main',
            # Hardware ESP32
            'motor_interface     = examen_4_2.motor_interface:main',
        ],
    },
)