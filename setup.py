from setuptools import setup

package_name = 'nav6d_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nav6d_sim maintainer',
    maintainer_email='user@example.com',
    description='Simulation utilities for nav6d planning and trajectory testing.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = nav6d_sim.mission_manager:main',
            'path_follower = nav6d_sim.path_follower:main',
            'path_pruner = nav6d_sim.path_pruner:main',
            'tf_bridge = nav6d_sim.tf_bridge:main',
            'trajectory_mode_generator = nav6d_sim.trajectory_mode_generator:main',
            'trajectory_sampler = nav6d_sim.trajectory_sampler:main',
            'nav_6d_optimize_traj = nav6d_sim.nav_6d_optimize_traj:main',
            'mavproxy_waypoint_loader = nav6d_sim.mavproxy_waypoint_loader:main',
        ],
    },
)
