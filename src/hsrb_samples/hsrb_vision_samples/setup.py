# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# Copyright (C) 2016 Toyota Motor Corporation

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['hsrb_vision_samples'],
    package_dir={'': 'src'})

setup(**setup_args)
