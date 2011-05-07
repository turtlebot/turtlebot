display: Android Joystick
description: Drive TurtleBot with an Android device without a camera feed and without maps
platform: turtlebot
launch: turtlebot_teleop/android_joystick.launch
interface: turtlebot_teleop/android_joystick.interface
icon: turtlebot_teleop/android-lightning-turtlebot.png
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.teleop.Teleop
   app: 
     gravityMode: 0
