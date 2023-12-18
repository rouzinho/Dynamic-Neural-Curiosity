# A ROS Initializer in Cedar
This is a DFT script to initialize ROS inside Cedar.

Everything you want to know about DFT -> https://dynamicfieldtheory.org/

Cedar is the C++ Framework implementing the concepts of DFT -> https://cedar.ini.rub.de/

Everything you need to know about ROS -> http://www.ros.org/

## Getting Started

The script is a basic ROS init() thread allowing to subscribe and publish to any topics with a DFT plugin.

It basically consists of a C++ code starting a ROS init() thread, then the plugin itself is listening to the topic. 

The code work for the 6.x version of Cedar.


### Prerequisites

You first need to install cedar by following the instructions here : https://github.com/cedar/cedar

You can't use a precompiled version of Cedar to compile and run the plugin.

I suggest reading about how to create a plugin in Cedar first, it will greatly help to understand how it works : https://cedar.ini.rub.de/tutorials/writing_custom_code_for_cedar/

Install ROS : http://wiki.ros.org/ROS/Installation

**Warning**

ROS comes with Qt5 and Cedar officially support only Qt4, so I suggest a basic installation of ROS.
For my case, I have Cedar and a basic ROS on a computer and a complete ROS desktop on another computer dedicated to my robot. I communicate between them through the network with the topics.

Anyway, ROS and Cedar are a bit to powerful to run on the same computer (if you have a big DFT model and a complex robot), so I recommend using 2 different computer.

**INSTALL YARP**

This last version of the plugin requires yarp (cedar built with yarp support - you don't have to do$

https://www.yarp.it/install.html

If you don't need it, remove the find_package(YARP REQUIRED) in the cedarProject.cmake

### Installing

First clone the repository :

`git clone https://github.com/rouzinho/RosInitCedar.git`

In the project.conf, change the CEDAR_HOME directory to your own :

`set(CEDAR_HOME "your_own_cedar_repository")`

Then create a build repository and prepare the environment for the compilation :

`mkdir build`

`cd build`

`cmake ..`

Finally start the compilation :

`make`

You should see the plugin under the name libInitRos.so in the build/ repository

## Run the plugin

Execute cedar and load it into cedar 

*Tools -> Manage plugins*

In the plugin Manager window, click on *add* and choose the plugin libInitRos.so. This one should appear in the window.

You can close the window. The script is loaded inside cedar and before loading it, make sure your ROS node is running.

Run the ROS init() thread 

*Scripting -> C++ Scripts...*

In the window, you should see the name of the thread inside the plugin (InitRos here). Click on the add button.
Then click on play to start the Ros initialisation. This will create a thread that will init ROS in order to subscribe to any topics you want.


You can now go back to the cedar main interface and click on the Utilities tab.


## Authors

Quentin Houbre - Tampere University of Technology

## License

This project is licensed under the BSD licence

# RosInitCedar
