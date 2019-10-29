# Securbot database model
A document based NoSQL database has been chosen for the project in the form of
MongoDB. As an exploration, we devise two structures for the data model, one
being normalized into different collections containing the different types of
data used in the application, the other denormalized into a document based model.

## Requirements
The database will store the following informations:
- List of the active robots in the network.
- List of patrols created by the user. (specific to a robot)
- List of schedules for a robot.
- List of event for a robot

## Normalized data model:
Using this module, we seperate the diffent types of data per collection and do
the linking using the fields.

### Collections of the database:
```
robots
patrols
schedules
events
```

### Robot document model:
```js
$jsonSchema:
{
    type: "object",
    required: ["name"],
    properties: {
        name: {
            type: "string",
            description: "Name of the robot"
        },
        platform: {
            type: "object",
            properties: {
                OS: { type: "string" },
                ROS: { type: "string" },
                CPU: { type: "string" },
                GPU: { type: "string" },
                Batteries: { type: "string" },
                Frame: { type: "string" }
            }
        }
    }
}
```

### Patrol document model:
```js
$jsonSchema:
{
    type: "object",
    required: ["robot", "name", "waypoints"],
    properties: {
        robot: {
            type: "objectId",
            description: "Robot for which the patrol is valid"
        }
        name: {
            type: "string",
            description: "Name of the patrol"
        },
        last_modified: { type: "timestamp" },
        description_text: { type: "string" },
        waypoints: {
            type: "array",
            items: {
                type: "object",
                description: "List of waypoints on the patrol",
                required: ["quaternion", "hold_time"],
                properties: {
                    quaternion: {
                        type: "object",
                        required: ["a", "b", "c", "d"],
                        properties: {
                            a: { type: "number" },
                            b: { type: "number" },
                            c: { type: "number" },
                            d: { type: "number" }
                        }
                    },
                    hold_time: { type: "number" }
                }
            }
        }
    }
}
```

### Schedule document model:
```js
$jsonSchema:
{
    type: "object",
    required: ["robot", "name", "patrol", "cron", "enabled"],
    properties: {
        robot: {
            type: "objectId",
            description: "Robot for which the schedule is valid"
        }
        name: {
            type: "string",
            description: "Name of the schedule"
        },
        last_modified: { type: "timestamp" },
        description_text: { type: "string" },
        patrol: {
            type: "objectId",
            description: "Patrol to execute at given time"
        },
        cron: {
            type: "string",
            description: "The time of execution expressed in crontab format"
        },
        timeout: { type: "number" },
        repetitions: {
            type: "number",
            description: "The number of times to repeat, if <=0 repeat forever"
        },
        enabled: {
            type: "bool",
            description: "Indicates to the robot it should execute this schedule"
        }
    }
}
```

### Event document model:
```js
$jsonSchema:
{
    type: "object",
    required: ["robot", "object", "context", "time", "location_quaternion", "tags", "alert", "viewed"],
    properties: {
        robot: {
            type: "objectId",
            description: "Robot involved in the event"
        }
        object: {
            type: "string",
            description: "Onliner announcing the event"
        },
        context: {
            type: "string",
            description: "Indicates the state of the robot at the momemnt of the
                event. Could hold information such as patrol or active waypoint"
        },
        time: { type: "timestamp" },
        description_text: { type: "string" },
        location_quaternion: {
            type: "object",
            required: ["a", "b", "c", "d"],
            properties: {
                a: { type: "number" },
                b: { type: "number" },
                c: { type: "number" },
                d: { type: "number" }
            }
        }
        tags: {
            type: "array",
            items: { type: "string" },
            description: "The strings used to filter event"
        },
        alert: {
            type: "bool",
            description: "Whether to produce a notification to the user"
        },
        viewed: {
            type: "bool",
            description: "Wheter a user has viewed this event"
        }
    }
}
```

## Denormalized data model
The denormalized data model would be similar, but there would only be one robot
collection. The other collections would be embbeded in their respective robot.
