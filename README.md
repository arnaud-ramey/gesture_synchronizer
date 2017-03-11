# gesture_synchronizer

[![Build Status](https://travis-ci.org/arnaud-ramey/gesture_synchronizer.svg)](https://travis-ci.org/arnaud-ramey/gesture_synchronizer)

The gesture architecture relies on two entities:

  - A set of joint_players, one for each degree of freedom
  - The gesture_synchronizer , in charge of making everybody synchronized

joint_player
============

In charge of playing a gesture at the level of a given joint (degree of freedom).
For instance, it will control the angle of a servo, or emit a sound.
This is a general interface for a gesture player.
It includes several mechanisms:
  - the subscription to the gesture topic

  - the advertising for the ack topic

gesture_synchronizer
=====================

This is a publisher for gesture messages.
When you publish a gesture, here is what it does in order:

1) It first publishes the message
  (gesture_msgs::KeyframeGesture) on the dedicated channel.
  When the gesture players receive it, they will go to the initial
  position of the gesture.
  The way they do it depends on the player:
  a voice player will wait for the voice to be available,
  while a servo will physically move to the initial angle.

2) It determines the number of acks (acknowledgement) needed to go on.
  It is determined by the number of subscribers to gestures
  at current time.
  As such, if a player dies, this number is decreased by one.

3) it waits for the acks for all joints.

4) when all acks are received, it emits a "play" order for all joints,
  giving them the order of starting the execution of gestures.

gesture GUI
===========

The gesture GUI is a powerful tool for creating and editing
gestures of type gesture_msgs::KeyframeGesture easily
and with a graphical interface.

On the one hand, it enables "playing" gestures by parsing XML files
and emitting them to the ROS architectures, and hence to
GesturePlayerRos players listening to this gestures.
A slider enables navigating through the gesture by simulating
the gesture time.

On the other hand, it subscribes to some degrees of freedom of the robot
via Qt "widgets".
The current implementation enables subscribing to Dynamixel servos
and ETTS (voice) skill.
The received messages of these degrees of freedom can be added to
the gesture_msgs::KeyframeGesture by clicking on a button.

How to install
==============

Dependencies from sources
-------------------------

Dependencies handling is based on the [wstool](http://wiki.ros.org/wstool) tool.
Run the following instructions:

```bash
$ sudo apt-get install python-wstool
$ roscd ; cd src
$ wstool init
$ wstool merge `rospack find gesture_synchronizer`/dependencies.rosinstall
$ wstool update
```

Dependencies included in the Ubuntu packages
--------------------------------------------

Please run the ```rosdep``` utility:

```bash
$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep install gesture_synchronizer --ignore-src
```

Build package with Catkin
-------------------------

```bash
$ catkin_make --only-pkg-with-deps gesture_synchronizer
```
