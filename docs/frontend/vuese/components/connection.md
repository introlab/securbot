# connection

Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>, Anthony Parris <anthony.parris@usherbrooke.ca>, File :  Connection.vue Desc :  Vue SFC used as a widget that shows the element in the array given in props in a html list. All list element are buttons that, when clicked, request a connection/emit an event on the bus (given in props). It manages the different state on the connection and change a badge to give visual feedback to the user on it. The array given should contains the name and the id of the easyRTC peers that are in the connected room. The component calling this one should manage the easyRTC part and the connection, this component only show peers and manage the connection selection. Dependencies : -Bootstrap-Vue

## Props

<!-- @vuese:connection:props:start -->
|Name|Description|Type|Required|Default|
|---|---|---|---|---|
|selfId|-|—|`false`|-|
|peersTable|-|—|`false`|-|
|bus|-|—|`false`|-|

<!-- @vuese:connection:props:end -->


## Events

<!-- @vuese:connection:events:start -->
|Event Name|Description|Parameters|
|---|---|---|
|peer-connection|-|-|

<!-- @vuese:connection:events:end -->


