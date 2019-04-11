# joystick

Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>, Valerie Gauthier <valerie.gauthier4@usherbrooke.ca>, File :  Joystick.vue Desc :  Vue SFC used as a widget that draws an joystick that the user can use to send teleoperation control to the robot that it is connected to. Takes 2 absolute values in props to set the max value of a command and a bus to send the event (new joystick value). Dependencies : -Bootstrap-Vue Note :  The original file comes from a project called SOSCIP and we modified it to fit this project.

## Props

<!-- @vuese:joystick:props:start -->
|Name|Description|Type|Required|Default|
|---|---|---|---|---|
|enable|-|—|`false`|-|
|absoluteMaxX|-|—|`false`|-|
|absoluteMaxY|-|—|`false`|-|
|bus|-|—|`false`|-|

<!-- @vuese:joystick:props:end -->


## Events

<!-- @vuese:joystick:events:start -->
|Event Name|Description|Parameters|
|---|---|---|
|joystick-position-change|-|-|

<!-- @vuese:joystick:events:end -->


