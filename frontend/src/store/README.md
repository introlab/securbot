# Store

- [Store](#store)
  - [Documentation](#documentation)
  - [The root store](#the-root-store)
  - [The database module](#the-database-module)
    - [Environment variables](#environment-variables)
    - [Root store interaction](#root-store-interaction)
      - [Module -&gt; Root](#module--gt-root)
        - [States](#states)
        - [Getters](#getters)
        - [Mutations](#mutations)
        - [Actions](#actions)
      - [Root -&gt; Module](#root--gt-module)
        - [States](#states-1)
        - [Getters](#getters-1)
        - [Mutations](#mutations-1)
        - [Actions](#actions-1)
      - [Components -&gt; Module](#components--gt-module)
        - [States](#states-2)
        - [Getters](#getters-2)
        - [Mutations](#mutations-2)
        - [Actions](#actions-2)
  - [The EasyRTC-client module](#the-easyrtc-client-module)
    - [Environment variables](#environment-variables-1)
    - [Root store interaction](#root-store-interaction-1)
      - [Module -&gt; Root](#module--gt-root-1)
        - [States](#states-3)
        - [Getters](#getters-3)
        - [Mutations](#mutations-3)
        - [Actions](#actions-3)
      - [Root -&gt; Module](#root--gt-module-1)
        - [States](#states-4)
        - [Getters](#getters-4)
        - [Mutations](#mutations-4)
        - [Actions](#actions-4)
      - [Components -&gt; Module](#components--gt-module-1)
        - [States](#states-5)
        - [Getters](#getters-5)
        - [Mutations](#mutations-5)
        - [Actions](#actions-5)
  - [Change a module or use another API](#change-a-module-or-use-another-api)

The SecurBot application uses Vuex as its data store. The choice to split the store into multiple files and modules was made to simplify readability and usability.
Vuex documentation can be found [here](https://vuex.vuejs.org/).

## Documentation

Currently, the entire store has been commented and this readme acts as its documentation, there is sadly no website, html pages or markdown (like there is with the components) that regroup and facilitate reading... The [jsdoc-vuex-plugin](https://www.npmjs.com/package/jsdoc-vuex-plugin) module could help with that but it needs to be implemented and configured...

## The root store

The root store is composed of 4 files, each file representing one of its "concepts". The folders are for the store modules. The index.js file imports everything.

___
## The database module

The database module is a store module that uses the request-promise package to request data from the database API.

### Environment variables
This module needs two environment variables to work:

- VUE_APP_SERVER_URL: the server URL where the database is situated.
- VUE_APP_DB_PATH: (optional) the path that follows the server URL to access the database API.

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
## The EasyRTC-client module

The EasyRTC-client module is a store module that uses the EasyRTC API to connect to a WebRTC server where it can call, interact and get data from the robots.

### Environment variables
This module needs two environment variables to work:

- VUE_APP_SERVER_URL: the server URL.
- VUE_APP_SERVER_ROOM_NAME: the name of the room to connect to.

### Root store interaction

#### Module -> Root
##### States
The EasyRTC-client module does not use any states from the root store.
##### Getters
The EasyRTC-client module does not use any getters from the root store.
##### Mutations
The EasyRTC-client module uses the following mutations from the root store:

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
The components do not use any getters from the module.
##### Mutations
The components do not use any mutations from the module.
##### Actions
The components use the following actions from the module:

- disconnectFromRobot
- connectToRobot

___

## Change a module or use another API
The current modules can easily be replaced and the implementation of a new module can be transparent for the rest of the code if it respects the calls mentioned above (and indirectly the data format of those).

For example, using another API than EasyRTC for the client will require the new module to include the same 6 states mentioned above with actions for connecting, disconnecting, sending data and resetting its video stream, plus have this module make calls to the setMapSize mutation and the updateHTMLVideoElements action. If the module includes those, the rest on its content/logic does not matter and its implementation should be transparent.
