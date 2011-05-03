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
    
# From: http://www.harmony-central.com/MIDI/Doc/table2.html
MIDI_TABLE = {'rest': 0, 'R': 0, 'pause': 0,
              'G1': 31, 'G#1': 32, 'A1': 33,
              'A#1': 34, 'B1': 35,

              'C2': 36, 'C#2': 37, 'D2': 38,
              'D#2': 39, 'E2': 40, 'F2': 41,
              'F#2': 42, 'G2': 43, 'G#2': 44,
              'A2': 45, 'A#2': 46, 'B2': 47,

              'C3': 48, 'C#3': 49, 'D3': 50,
              'D#3': 51, 'E3': 52, 'F3': 53,
              'F#3': 54, 'G3': 55, 'G#3': 56,
              'A3': 57, 'A#3': 58, 'B3': 59,

              'C4': 60, 'C#4': 61, 'D4': 62,
              'D#4': 63, 'E4': 64, 'F4': 65,
              'F#4': 66, 'G4': 67, 'G#4': 68,
              'A4': 69, 'A#4': 70, 'B4': 71,

              'C5': 72, 'C#5': 73, 'D5': 74,
              'D#5': 75, 'E5': 76, 'F5': 77,
              'F#5': 78, 'G5': 79, 'G#5': 80,
              'A5': 81, 'A#5': 82, 'B5': 83,

              'C6': 84, 'C#6': 85, 'D6': 86,
              'D#6': 87, 'E6': 88, 'F6': 89,
              'F#6': 90, 'G6': 91, 'G#6': 92,
              'A6': 93, 'A#6': 94, 'B6': 95,

              'C7': 96, 'C#7': 97, 'D7': 98,
              'D#7': 99, 'E7': 100, 'F7': 101,
              'F#7': 102, 'G7': 103, 'G#7': 104,
              'A7': 105, 'A#7': 106, 'B7': 107,

              'C8': 108, 'C#8': 109, 'D8': 110,
              'D#8': 111, 'E8': 112, 'F8': 113,
              'F#8': 114, 'G8': 115, 'G#8': 116,
              'A8': 117, 'A#8': 118, 'B8': 119,

              'C9': 120, 'C#9': 121, 'D9': 122,
              'D#9': 123, 'E9': 124, 'F9': 125,
              'F#9': 126, 'G9': 127}

