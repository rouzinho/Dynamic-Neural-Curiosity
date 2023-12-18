# PosToField

Plugin that fit an object position in the robot space to the neural field space. The transformation consists of a simple linear scaling.
The plugin listen to a ROS topic that contains a list of all visible objects.

# Params

```
lower space x : lower bound of the robot space on the x component
upper space x : upper bound of the robot space on the x component
lower space y : lower bound of the robot space on the y component
upper space y : upper bound of the robot space on the y component
size field x : size of the neural field on x
size field y : size of the neural field on y
topic name objects : ROS topic to listen that delivers a list of objects with their location in the robot space.
```