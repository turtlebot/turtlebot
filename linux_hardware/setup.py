#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['linux_hardware'],
    package_dir={'':'src'},
    scripts=['scripts/laptop_battery.py'],
    requires=['diagnostic_msgs','rospy','std_msgs']
)

setup(**d)
