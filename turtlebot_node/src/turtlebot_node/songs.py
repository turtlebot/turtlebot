#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

def bonus(robot):
    # a nice bit of goodness from the turtlebot driver by Xuwen Cao and
    # Morgan Quigley
    song = (
        (76, 16), (76, 16), (72, 8),  (76, 16), 
        (79, 32), (67, 32), (72, 24), (67, 24), 	
        (64, 24), (69, 16), (71, 16), (70, 8), 
        (69, 16), (79, 16), (76, 16), (72, 8), 
        (74, 16), (71, 24), (60, 16), (79, 8), 
        (78, 8),  (77, 8),  (74, 8),  (75, 8), 
        (76, 16), (67, 8),  (69, 8),  (72, 16), 
        (69, 8),  (72, 8),  (74, 8),  (60, 16), 	
        (79, 8),  (78, 8),  (77, 8),  (74, 8), 
        (75, 8),  (76, 16), (76, 4),  (78, 4), 
        (84, 16), (84, 8),  (84, 16), (84, 16), 
        (60, 16), (79, 8),  (78, 8),  (77, 8), 
        (74, 8),  (75, 8),  (76, 16), (67, 8), 
        (69, 8),  (72, 16), (69, 8),  (72, 8), 
        (74, 16), (70, 4),  (72, 4),  (75, 16), 
        (69, 4),  (71, 4),  (74, 16), (67, 4), 
        (69, 4),  (72, 16), (67, 8),  (67, 16), 
        (60, 24),
         )
    # have to make sure robot is in full mode
    robot.sci.send([128, 132])
    robot.sci.send([140, 1, len(song)])
    for note in song:
        robot.sci.send(note)
    robot.sci.play_song(1)
    

