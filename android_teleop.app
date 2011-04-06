display: Android Joystick
description: Control the TurtleBot with an Android device
platform: turtlebot
launch: turtlebot_teleop/android_teleop.launch
interface: turtlebot_teleop/android_teleop.interface
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.teleop.Teleop
   app: 
     gravityMode: 0
