import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'px4_gripper_missions'


def data_files_for(subdir):
    """(install_path, [source_files]) pairs for every file under config/<subdir>,
    preserving the subdirectory structure under share/<package>/<subdir>."""
    pairs = []
    for dirpath, _dirnames, filenames in os.walk(os.path.join('config', subdir)):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, 'config')
        install_path = os.path.join('share', package_name, 'config', rel)
        sources = [os.path.join(dirpath, f) for f in filenames]
        pairs.append((install_path, sources))
    return pairs


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        *data_files_for('scenarios'),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='px4_gazebo_sim_setup',
    maintainer_email='todo@todo.todo',
    description=(
        "Scripted mission programs for Scenarios.md's grasp scenarios "
        '(SC-01..SC-05).'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_scenario = px4_gripper_missions.run_scenario:main',
            'run_experiment = px4_gripper_missions.experiment:main',
            'export_trials = px4_gripper_missions.export_trials:main',
            'build_windows = px4_gripper_missions.windows:main',
        ],
    },
)
