# Iterative Boost in Cedar

This Cedar plugin induce an iterative boost. At each time step, an increasing boost is produced until a certain threshold.
The widget takes two nodes as input. So basically, a boost rises until it receives an activation from a stop node. When the reset node is activated, the boost is reset to 0 and start rising again.
It's pratical if you want to slowly rise the resting level of a Neural field and see the higher activation peaks. The input node stops the rise of the resting level, so easy to integrate in any architecture.

Everything you want to know about DFT -> https://dynamicfieldtheory.org/

Cedar is the C++ Framework implementing the concepts of DFT -> https://cedar.ini.rub.de/


## Getting Started

The plugin is a widget reading two input node (reset and stop) and producing a rising boost.

The code work for the 6.x version of Cedar.


### Prerequisites

You first need to install cedar by following the instructions here : https://cedar.ini.rub.de/tutorials/installing_and_running_cedar/

You can't use a precompiled version of Cedar to compile and run the plugin.

I suggest reading about how to create a plugin in Cedar first, it will greatly help to understand how it works : https://cedar.ini.rub.de/tutorials/writing_custom_code_for_cedar/


### Installing

First clone the repository :

`https://github.com/rouzinho/IterativeBoost.git`

In the project.conf, change the CEDAR_HOME directory to your own :

`set(CEDAR_HOME "your_own_cedar_repository")`

Then create a build repository and prepare the environment for the compilation :

`mkdir build`

`cd build`

`cmake ..`

Finally start the compilation :

`make`

You should see the plugin under the name libEarListener.so in the build/ repository

## Run the plugin

Execute cedar and load it into cedar 

*Tools -> Manage plugins*

In the plugin Manager window, click on *add* and choose the plugin libIterativeBoost.so (located in build/). This one should appear in the window.

You can close the window. The plugin is loaded inside cedar.

You can now go back to the cedar main interface and click on the Utilities tab.


## Authors

Quentin Houbre - Tampere University.

## License

This project is licensed under the BSD licence


