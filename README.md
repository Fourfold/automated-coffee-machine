# Robotics Assignment - Session 12

## Behavior Trees with ROS2 - Firas Dimashki


To build and run the project, enter the following commands while in the directory of the repository:
```
source install/setup.bash
colcon build
ros2 run brewer brewer_node
```
---
When the project is launched, the coffee machine is ready to recieve user input.

There are many ways to send your input to the machine. The easiest way to do that is by launching `rqt` in your terminal.
```
rqt
```
When the GUI launches, navigate to Plugins -> Topics -> Message Publisher. You can see there are 2 topics related to the coffee machine: `/brew` and `/clean`.

The `/brew` topic is used to enter the type of coffee to brew. The `/clean` topic is used to send a boolean to the machine which specifies whether the machine should clean after brewing the coffee or not.

The supported coffee types that can be sent through the `/brew` topic are: `Espresso`, `Cappucino`, and `Latte`. If neither of these coffee types are entered, the coffee machine will not respond to your input.
