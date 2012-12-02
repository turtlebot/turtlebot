#! /usr/bin/env python

"""
usage: %prog [args]
"""

import roslib
roslib.load_manifest('turtlebot_app_manager')

import os, sys, string
from optparse import OptionParser

import app_manager

def main(argv, stdout, environ):

  parser = OptionParser(__doc__.strip())
  (options, args) = parser.parse_args()

  try:
    a = turtlebot_app_manager.App(args[0])
    print a.yaml
  except turtlebot_app_manager.TaskException, e:
    print e

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
