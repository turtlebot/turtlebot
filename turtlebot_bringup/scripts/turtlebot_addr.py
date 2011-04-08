#!/usr/bin/env python
# warning: we don't express a rosdep on netifaces.  This is part of the turtlebot image
import sys
import netifaces

for inf in netifaces.interfaces():
  if inf.startswith('wlan'):
    addrs = netifaces.addresses(inf)
    if not netifaces.IF_INET in addrs:
      continue
    else:
      print addrs[netifaces.IF_INET]['addr']
      break
else:
  print >> sys.stderr, "failed to determine IP address"
  print "127.0.0.1" #bind to loopback for now
  sys.exit(1)
