# Changelog
> Changelog file for securbot frontend. Created on September 17th on version 0.1.0.

## SEC-500
> Version: 0.2.0

- Added easyrtc api calls into the store. The store is the only thing using the api and the components need to use actions (dispatches) to ask the store to call the api. What has been changed for that:
  - Layout:
    - Removed all methods
  - Navbar:
    - New Connection component
    - Removed the bus prop requirement
  - Connection:
    - Removed the bus prop requirement
    - Removed the handleConnectionChanged method
    - Rename handlePeerConnection to handleConnection
    - Removed disconnectFromRobot method, replaced it by a dispatch
    - Replaced the bus calls by commits and dispatched
    - Removed connectToRobot method, replaced it by a dispatch
  - Store:
    - Integrated Easyrtc api inside the store, created the required actions and mutations for it to work.
    - Added a darkMode state to be used/implemented in the future
  - Teleop:
    - Removed easyrtc events
    - Integrated Vuex
    - Changed components calls in html
  - Events:
    - Removed the Vue event
  - Patrol:
    - Removed easyrtc events
    - Integrated Vuex
    - Changed components calls in html
  - Joystick:
    - Now dispatches to the store its data to be sent to the robot
  - PatrolMap:
    - Removed easyrtc events
    - Integrated Vuex
  - SaveLoad:
    - Removed easyrtc events
    - Integrated Vuex
  - WaypointTable:
    - Removed easyrtc events
    - Integrated Vuex
- Rename the **pages** folder to **views**

## SEC-510
> Version: 0.1.1

- Added Navbar component, replaced the code in layout by a call the to compenent.
- Revert the patrol map id and fix and fixed a typo inside the getVideoOffsetAndScale method.

## v0.1.0
> 17-09-2019

- Creation of the changelog file.
- Added changelog and Env Variables sections to README.
