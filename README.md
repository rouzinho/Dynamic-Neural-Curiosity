# Dynamic Neural Curiosity

## Overview

### Description

This repository gathers all necessary packages and source code to run the dynamic neural curiosity architecture.

<div style="display:flex">
     <div style="flex:1;padding-right:10px;">
       <img src="https://github.com/rouzinho/Dynamic-Neural-Curiosity/assets/10597250/d95a08f5-bc72-45d9-86d8-2d89cb74d05c" width="395"/>
        <img src="https://github.com/rouzinho/Dynamic-Neural-Curiosity/assets/10597250/bf68ae0f-73ec-431c-b334-4f51edb7af49" width="372"/>
     </div>
</div>

The purpose of the experiment is to root attention and curiosity together by taking inspiration from neuroscience, more especially about the role of the Locus Coeruleus. The architecture is a robotics implementation where a robot arm continuously switch between goal discovery (exploration) and learning (exploitation). The robot first discovers new goals through bottom-up attention, then select and monitor the learning of these goals through dynamic curiosity. We call here dynamic curiosity the evolution of the forward model error through a learning progress. The complete process is modelled with Dynamic Neural Fields, leveraging interesting properties regarding the selection of goals as well as the learning dynamics that can occur. You can find complete results in the paper.

### Results

![test](https://github.com/rouzinho/Dynamic-Neural-Curiosity/assets/10597250/e9fd152f-51fc-49b4-8259-b81a09694f60)

The small GIF is a reduced example of the demo file in the video folder. The demo file has subtitles to provide more description.

## Reproduce the experiment

The experiment is done across 2 computers : one running the panda simulation along with the robot controller with ROS melodic. The other computer run the DNFs and the other packages.

### Prerequisites

Install the panda simulator from here (and many thanks for the access) : https://github.com/erdalpekel/panda_simulation

We provide in the Gazebo folder the model of the objects used for the experiment. For ease of use, we integrate a script to each of them that delivers their location in order to replace the camera sensor as seen in the paper. This purpose is only to simplify the reproduction of the experiment and does not have any incidence on the results.

Install the Cedar software to simulate the dynamic neural fields : https://cedar.ini.rub.de/   You need to compile the software in order to use custom plugins.

Place these plugins for Cedar in a repository of your choice and adjust the CMakeList inside each of them to fit the install folder of Cedar :

```
RosInit
ErrorSubscriber
IterativeBoost
PosToField
Preshape
RosBool
RosDatas
RosFloat
RosFloatPublisher
RosGoalContPub
RosGoalPublisher
RosHebbian
RosMotor2D
RosScene
```

Then, individually compile each package by going into the build folder inside them and compile (example with ErrorSubscriber) :

```
cd ErrorSubscriber
cd build
cmake ..
make
```

You can now open Cedar and load the ROS node script that with create a node able to publish and subscribe to ROS topic. To do so : Tools -> Manage Plugins -> add..  Then add the libRosInit.so inside the RosInitCedar  plugin. To start the ROS node, go to Scripting ->  C++ scripts -> select InitRos in the list then add and click on the play button. You master ROS node must be running prior to these steps.

For the other plugins, you can import them as described before, except they are not just script to start but elements presents in 




