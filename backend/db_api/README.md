# Securbot data API
This is the database API used by the webpage and the robot to
store data.

## Installation
```
$ npm install
```

## Usage
A MongoDB instance is required.

Setup the following environment varialbles or use their default values:
- **API_PATH**:
default: `/`
- **API_PORT**:
default: `8080`
- **DB_HOST**:
default: `localhost`
- **DB_PORT**:
default: `27017`

Launch the application:
```
$ node .
```


## API
The endpoints and their usage

### Files:
Allows to store files upoaded by the robots

##### POST /API_PATH/files
The request body should be of type `form-data` and contain a single file key
named `file`

The request response will contain the id of the file:
```js
{
    "success": true,
    "_id": "5dd41a39f9080f0014249898",
    "file": {
        "fieldname": "file",
        "originalname": "example.png",
        "encoding": "7bit",
        "mimetype": "image/png",
        "id": "5dd41a39f9080f0014249898",
        "filename": "8cc77539e533ae84b2b23951bcafe447",
        "metadata": null,
        "bucketName": "fs",
        "chunkSize": 261120,
        "size": 11871164,
        "md5": "2de9925ba351392765b2586910a1958f",
        "uploadDate": "2019-11-19T16:37:13.657Z",
        "contentType": "image/png"
    }
}
```

##### GET /API_PATH/files/`fileId`
Returns a binary chunked stream with `Content-Type` set to the content type
of the uploaded file


### Robots:
Example robot document:
```js
{
    name: "Robot Name" // required
    platform: {
        OS: "Ubuntu 16.04",
        ROS: "Kinetic",
        CPU: "Cortex-A57 MPCore",
        GPU: "Nvidia Pascal",
        Batteries: "40000 mAh",
        Frame: "turtlebot"
    }
}
```
Complete schema can be found in `api/models/robot.js`

##### GET /API_PATH/robots
Returns the list of all robots

##### POST /API_PATH/robots
Upload the robot configuration contained in request body.
Request body should follow schema defined in `api/models/robot.js`.
Returns the robot object with assigned `_id`.

##### GET /API_PATH/robot/`robotId`
Returns the complete robot document associated with `robotId`.

##### PUT /API_PATH/robot/`robotId`
Update the robot document associated with `robotId`.
Request body should follow schema defined in `api/models/robot.js`.
Returns the updated robot document.


### Patrols:
Example of patrol document:
```js
{
    robot: "5d9adfbcaba84367805737d8", // required
    name: "Patrol 1", // required
    description_text: "text",
    last_modified: "2019-10-07T06:52:19.831Z",
    waypoints: [ // required
        { coordinate: {x: 0, y: 0, yaw: 180}, hold_time_s: 10 },
        { coordinate: {x: 0, y: 0, yaw: 180}, hold_time_s: 10 },
        { coordinate: {x: 0, y: 0, yaw: 180}, hold_time_s: 10 }
    ]
}
```
Complete schema can be found in `api/models/patrol.js`

##### GET /API_PATH/robots/`robotId`/patrols
Returns the list of all patrols saved for the specified `robotId`.

##### POST /API_PATH/robots/`robotId`/patrols
Upload the patrol plan contained in request body.
Request body should follow the schema defined in `api/models/patrol.js`.
Returns the patrol object with assigned `_id`.

##### GET /API_PATH/robot/`robotId`/patrols/`patrolId`
Returns the complete patrol document associated with `patrolId`.

##### PUT /API_PATH/robot/`robotId`/patrols/`patrolId`
Update the patrol document associated with `patrolId`.
Request body should follow schema defined in `api/models/patrol.js`.
Returns the updated patrol document.

### Schedules:
Example of schedule document:
```js
{
    robot: "5d9adfbcaba84367805737d8", // required
    name: "Schedule 1", // required
    description_text: "text",
    last_modified: "2019-10-07T06:52:19.831Z",
    patrol: "5d9aea6e685a0c34f6015a3a", // required
    cron: "5 4 * * *", // required
    timeout_s: 3600,
    repetitions: 3,
    enabled: true // required
}
```
Complete schema can be found in `api/models/schedule.js`

##### GET /API_PATH/robots/`robotId`/schedules
Returns the list of all schedules saved for the specified `robotId`.

##### POST /API_PATH/robots/`robotId`/schedules
Upload the schedule plan contained in request body.
Request body should follow the schema defined in `api/models/schedule.js`.
Returns the schedule object with assigned `_id`.

##### GET /API_PATH/robot/`robotId`/schedules/`scheduleId`
Returns the complete schedule document associated with `scheduleId`.

##### PUT /API_PATH/robot/`robotId`/schedules/`scheduleId`
Update the schedule document associated with `scheduleId`.
Request body should follow schema defined in `api/models/schedule.js`.
Returns the updated schedule document.

### Events:
Example of event document:
```js
{
    robot: "5d9adfbcaba84367805737d8", // required
    object: "Event 1", // required
    description_text: "text",
    context: "text",
    time: "2019-10-07T06:52:19.831Z",
    coordinate: {x: 0, y: 0, yaw: 180},
    tags: ["red", "important", "category"],
    alert: false,
    viewed: false
}
```
Complete schema can be found in `api/models/event.js`

##### GET /API_PATH/robots/`robotId`/events
Returns the list of all events saved for the specified `robotId`.
This `GET` request also accepts filters as part of it's body or parameter.
The full speficication for these filters is in `api/validators/event_filter`.
All the filter options are optionnal. Providing none of them with list all events.
The following filters are available:
 - `tag_and`: Tag array that must all be present in event to be listed
 - `tag_or`: Tag array, one of these must be present in event to be listed
 - `tag_not`: Tag array, none of these will be present in listed events
 - `search_expression`: String, Performs textual search on the event's text fields and tags
 - `alert`: Boolean, Filters for events that raise an alert or not
 - `viewed`: Boolean, Filters for events that have/haven't been marked as read
 - `before`: Date, only list events before this time
 - `afer` : Date, only list events after this time
 - `minimized` : Boolean, if true, only the `object`, `time`, `alert` and `viewed` fields will be listed

##### POST /API_PATH/robots/`robotId`/events
Upload the event contained in request body.
Request body should follow the schema defined in `api/models/event.js`.
Returns the event object with assigned `_id`.

##### GET /API_PATH/robot/`robotId`/events/`eventId`
Returns the complete event document associated with `eventId`.

##### PATCH /API_PATH/robot/`robotId`/schedules/`eventId`?viewed=true
Caller must set the `viewed` paramter to `true`.
This endpoint is specifically there to mark events as read.
