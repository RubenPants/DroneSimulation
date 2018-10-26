
# Overview

A simulation of a drone which will fly in a three dimensional virtual testbed. An array of unit cubes will be loaded inside the virtual testbed, the main
objective of the drone is to collect all cubes as fast as possible (no order specified). 



# Run the Project

To run the project, and thus open the testbed's Graphical User Interface, go to 'Virtual Testbed' -> 'src' -> 'main' -> 'MainLoop' and run 'MainLoop' as a 
Java Application.



# Virtual Testbed

The _virtual testbed_ will provide a clear representation of the drone and its actions. Inside the GUI there are multiple windows which will add a corresponding
functionality to the project. A quick overview of these windows are:  
* __File__ - Add a custom path to the testbed  
* __Settings__ - Change the drone or the testbed settings
* __Run__ - Manage the flow of the program (start, reset, run tests, ...)
* __Window__ - Change the drone-view
* __Inputs__ - Standard information and settings of the testbed and drone
* __Configuration__ - Change the drone's configuration
* __Configure Path__ - Configure a given or custom path to load in the testbed



# Automatic Running Tests

It is possible to test the drone on randomly generated paths. Go inside the testbed's GUI to 'Run' -> 'Run tests', a new window will pop up. Within the _testing
frame_ you can manage the tests: change the amount of tests, change the speed of the drone during the tests, change the maximum time the drone has to accomplish
one testing path, and more.



# Changing the AutoPilot

When changes are made within the _AutoPilot_, you have to export the whole _AutoPilot_ file as a jar and place this jar in the _Virtual Testbed_ file on the
following location: 'Virtual Testbed' -> 'lib' -> 'jar'. You __must__ name this jar 'autopilot.jar' otherwise the testbed will not recognize the jar. At the
moment there is no functionality to toggle between multiple jars, and thus it will not be possible to test or compare two or more autopilots at the same time.



# Continuation of the Project

This project is the first part of a larger whole.

TODO: Links, summary of the other projects, ...
