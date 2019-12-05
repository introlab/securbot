# Store

The SecurBot application uses Vuex as its data store. The choice to split the store into multiple files and modules was made to simplify readability and usability.
Vuex documentation can be found [here](https://vuex.vuejs.org/).

## The root store

The root store is composed of 4 files, each file representing one of its "concept". The folders are for the store modules. The index.js file imports everything.

___
## The database module

The database module is a store module that uses the request-promise package to request data from the database API.

### Environment variables
This module needs two environment variables to work:

- VUE_APP_SERVER_URL: the server url where the database is situated.
- VUE_APP_DB_PATH: (optionnal) the path that follows the server url to access the database API.

### Root store interaction

#### Module -> Root
##### States
The database module does not use any states from the root store.
##### Getters
The database module does not use any getters from the root store.
##### Mutations
The database module uses the following mutations from the root store:

- clearPatrols
- addPatrols
- clearSchedules
- addSchedules
- setCurrentPatrolId
- setCurrentScheduleId
- clearCurrentPatrol
- setCurrentPatrol
- clearCurrentSchedule
- setCurrentScheduleId
- setCurrentSchedule
  
##### Actions
The database module does not use any actions from the root store.
  
#### Root -> Module
##### States
The root store uses the following states from the module:

- robots
- events

##### Getters
The root store does not use any getters from the root store.
##### Mutations
The root store does not use any getters from the root store.
##### Actions
The root store does not use any getters from the root store.

#### Components -> Module
##### States
The components use the following states from the module:

- predefFilters
- eventFilter
- eventList
- eventImageURL
- robots
- tagList
- querying
- queryError
- display
- selectedRobot

##### Getters
The components use the following getters from the module:

- uri

##### Mutations
The components use the following mutations from the module:

- clearEventImageURL
- setEventFilter
- setRobotFilter
- setEventFilter
- resetQuery
- resetEvents

##### Actions
The components use the following actions from the module:

- filterEvents
- setEventImageURL
- savePatrol
- queryPatrols
- getPatrol
- removePatrol
- saveSchedule
- querySchedules
- getSchedule
- removeSchedule

___
## The easyrtc-client module

The easyrtc-client module is a store module that uses the easyrtc api to connect to a webrtc server where it can call, interact and get data from the robots.

### Environment variables
This module needs two environment variables to work:

- VUE_APP_SERVER_URL: the server url where the database is situated.
- VUE_APP_SERVER_ROOM_NAME: the name of the room the ui needs to connect to.

### Root store interaction

#### Module -> Root
##### States
The easyrtc-client module does not use any states from the root store.
##### Getters
The easyrtc-client module does not use any getters from the root store.
##### Mutations
The easyrtc-client module uses the following mutations from the root store:

- setMapSize
  
##### Actions
The easyrtc-client module uses the following actions from the root store:

- updateHTMLVideoElements
  
#### Root -> Module
##### States
The root store uses the following states from the module:

- robotId

##### Getters
The root store does not use any getters from the module.
##### Mutations
The root store does not use any getters from the module.
##### Actions
The root store uses the following actions from the module:

- sendData
- resetStreams

#### Components -> Module
##### States
The components use the following states from the module:

- myId
- robotId
- robotList
- connectionState
- robotStatus
- isDataChannelAvailable

##### Getters
The components does not use any getters from the module.
##### Mutations
The components does not use any mutations from the module.
##### Actions
The components use the following actions from the module:

- disconnectFromRobot
- connectToRobot

___

## Change a module or use an other API
The current modules are easily replaceable and the implementation of a new module can be transparent for the rest of the code if it respects the calls mentionned above (and indirectly the format that those use).

For example, using an other api than easyrtc for the client will required the new module to include the same 6 states mentionned above with actions for connecting, disconnecting, sending data and reseting its video stream  and also have this module make calls to the setMapSize mutations and the updateHTMLVideoElements. If the module includes those, the rest on its content/logic do not matter for the rest of the code and it implementation should be transparent.
