# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# Copyright (C) 2016 Toyota Motor Corporation

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['hsrb_mounted_devices_samples',
              'hsrb_mounted_devices_samples.force_torque_sensor',
              'hsrb_mounted_devices_samples.status_led',
              'hsrb_mounted_devices_samples.suction_mechanism'],
    package_dir={'': 'src'})

setup(**setup_args)
