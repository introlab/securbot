# teleop-page

Description : Component used as a page for teleoperation of the robot. It manages the layout of its components and communicate with its parent component through a bus given in props. The components used in this page are 2 VideoBox and 1 Joystick. This component have the following dependency : VideoBox.vue Component, Joystick.vue Component and Bootstrap-Vue for styling.

## Props

<!-- @vuese:teleop-page:props:start -->
|Name|Description|Type|Required|Default|
|---|---|---|---|---|
|bus|Vue bus use to communicate events to the other components|`Vue`|`true`|-|
|router|Vue bus use to emit events to the parent component for routing purposes|`Vue`|`true`|-|

<!-- @vuese:teleop-page:props:end -->


## Events

<!-- @vuese:teleop-page:events:start -->
|Event Name|Description|Parameters|
|---|---|---|
|destroyed|Destroyed event|Does not take any parameter|
|mounted|Triggered when button is clicked|This event doesn't emit any argument|

<!-- @vuese:teleop-page:events:end -->


## Methods

<!-- @vuese:teleop-page:methods:start -->
|Method|Description|Parameters|
|---|---|---|
|init|Init function for the teleop page|Does not take any parameter|
|changeJoystickState|Joystick state event callback, used to change the joystick state|The argument is a boolean representing the state|

<!-- @vuese:teleop-page:methods:end -->


