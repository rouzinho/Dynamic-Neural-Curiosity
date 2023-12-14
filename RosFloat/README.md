# Motor Control in Cedar

This Cedar plugin allow to send motor commands through YARP

Everything you want to know about DFT -> https://dynamicfieldtheory.org/

Cedar is the C++ Framework implementing the concepts of DFT -> https://cedar.ini.rub.de/

Everything you need to know about YARP https://www.yarp.it/

## Getting Started

The plugin is a widget reading outputs from a Neural Field and publishing the datas to a yarp topic.

You can define directly in the Qt Widget the destination port.

Of course you can adapt it to publish commands to any topics, but you might want to change the scale or format of the datas received from the Neural Field.

The code work for the 6.x version of Cedar.


### Prerequisites

You first need to install cedar by following the instructions here : https://cedar.ini.rub.de/tutorials/installing_and_running_cedar/

You can't use a precompiled version of Cedar to compile and run the plugin.

I suggest reading about how to create a plugin in Cedar first, it will greatly help to understand how it works : https://cedar.ini.rub.de/tutorials/writing_custom_code_for_cedar/

The code was tested with cedar and the iCub Simulator
### Installing

First clone the repository :

`https://github.com/rouzinho/Motor2DYarp.git`

In the project.conf, change the CEDAR_HOME directory to your own :

`set(CEDAR_HOME "your_own_cedar_repository")`

Then create a build repository and prepare the environment for the compilation :

`mkdir build`

`cd build`

`cmake ..`

Finally start the compilation :

`make`

You should see the plugin under the name libMotorTwoDim.so in the build/ repository

## Run the plugin

Execute cedar and load it into cedar 

*Tools -> Manage plugins*

In the plugin Manager window, click on *add* and choose the plugin libMotor.so (located in build/). This one should appear in the window.

You can close the window. The plugin is loaded inside cedar and before loading it, make sure your ROS node is running.

You can now go back to the cedar main interface and click on the Utilities tab.

Drag the MotorHead widget into the architecture panel. Connect the output of a space to rate widget to the input of the MotorHead widget. The outputs of the Neural Field now drive the motor of your choice !


## Work in progress


The plugin is more like an artefact binding DFT to the iCub robot.
Work in progress to use Qt elements to control the settings.



## Authors

Quentin Houbre - Tampere University

## License

This project is licensed under the BSD licence


