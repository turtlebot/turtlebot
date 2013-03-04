#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['scripts/laptop_battery.py'],
    requires=['diagnostic_msgs','rospy','std_msgs']
)

setup(**d)
