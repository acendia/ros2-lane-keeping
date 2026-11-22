"""Package installation for auto_drive_sim."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'auto_drive_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'models', 'road'),
         [path for path in glob('models/road/*') if os.path.isfile(path)]),
        (os.path.join('share', package_name, 'models', 'road', 'meshes'),
         glob('models/road/meshes/*')),
        (os.path.join('share', package_name, 'models', 'road_markings'),
         [path for path in glob('models/road_markings/*')
          if os.path.isfile(path)]),
        (os.path.join('share', package_name, 'models', 'road_markings',
                      'meshes'),
         glob('models/road_markings/meshes/*')),
        (os.path.join('share', package_name, 'models', 'road_two_lane'),
         [path for path in glob('models/road_two_lane/*')
          if os.path.isfile(path)]),
        (os.path.join('share', package_name, 'models', 'road_two_lane',
                      'meshes'),
         glob('models/road_two_lane/meshes/*')),
        (os.path.join('share', package_name, 'models', 'car'),
         [path for path in glob('models/car/*') if os.path.isfile(path)]),
        (os.path.join('share', package_name, 'models', 'lead_car'),
         [path for path in glob('models/lead_car/*')
          if os.path.isfile(path)]),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evangelos Kostakis',
    maintainer_email='evangelos.kostakis@example.com',
    description='Camera-based lane following simulation with Gazebo Harmonic.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'evaluator = auto_drive_sim.evaluator:main',
            'lane_controller = auto_drive_sim.lane_controller:main',
            'intersection_controller = auto_drive_sim.intersection:main',
            'lane_perception = auto_drive_sim.lane_perception:main',
            'lead_detector = auto_drive_sim.lead_detector:main',
            'lead_driver = auto_drive_sim.lead_driver:main',
            'teleop = auto_drive_sim.teleop:main',
        ],
    },
)
