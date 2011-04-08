#!/bin/bash

# Set the volume to max
amixer -c 0 sset PCM,0 100%
amixer -c 0 sset Master,0 100%

# Then moo.
aplay cow.wav
