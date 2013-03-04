#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['turtlebot_app_manager'],
    package_dir={'':'src'},
    scripts=['scripts/app_manager',
             'scripts/appmaster',
             'scripts/rosget',
             'scripts/test_app.py'
            ],
    requires=['rospy','rosgraph','roslaunch','rosunit']
)

setup(**d)
