## Structure reference

### Controls
`GLOBAL controls IS LEXICON().`

Variable of type `LEXICON`.
Defines vehicle behavior in phases 0 and 1 (pre-launch and atmospheric ascent).
List of possible keys:

Key                | Units | Opt/req  | Meaning
---                | ---   | ---      | ---
launchTimeAdvance  | s     | required | Launch time will be scheduled that many seconds before the launch site rotates directly under the target orbit
verticalAscentTime | s     | required | After liftoff, vehicle will fly straight up for that many seconds before pitching over
pitchOverAngle     | deg   | required | Vehicle will pitch over by that many degrees away from vertical
upfgActivation     | s     | required | The active guidance phase will be activated that many seconds after liftoff
launchAzimuth      | deg   | optional | Overrides automatic launch azimuth calculation, giving some basic optimization capability\*
initialRoll        | deg   | optional | Angle to which the vehicle will roll during the initial pitchover maneuver (default is 0)

\* - see notes to [`mission`](#mission) struct.

### Vehicle
`GLOBAL vehicle IS LIST().`

Variable of type `LIST` containing `LEXICON`s.
Each element of the list contains physical descriptions of each stage to be guided with UPFG, as well as desciption of the staging sequence.
The following table gives list of all possible keys each element of the list can have, some of which are required, others optional.

Key              | Type/units | Opt/req   | Meaning
---              | ---        | ---       | ---
name             | `string`   | required  | Name of the stage that will be printed in the terminal
massTotal        | kg         | optional* | Total mass of the **entire vehicle** at the moment of ignition of this stage
massFuel         | kg         | optional* | Mass of the fuel in this stage at the moment of ignition
massDry          | kg         | optional* | `massTotal` - `massDry`
gLim             | G          | optional  | Acceleration limit to be imposed on this stage (requires throttling engines)
minThrottle      | (0.0-1.0)  | optional**| Minimum possible throttle of this stage's engines (for Realism Overhaul)
throttle         | (0.0-1.0)  | optional  | Nominal throttle for this stage's engines (default = 1.0)
shutdownRequired | `boolean`  | optional  | Do this stage's engines need explicit shutdown upon activation of the next stage?\*\*\*
engines          | `list`     | required  | Parameters of each engine in the stage (details further)
staging          | `lexicon`  | required  | Description of method of activation of this stage (details further)

\* - of the three fields, `massTotal`, `massFuel` and `massDry`, one can be skipped, but **two** have to be given.  
\*\* - required if `gLim` is given.  
\*\*\* - in normal usage this has no effect, since PEGAS schedules staging exactly in the moment the stage runs out of fuel.
The purpose of this option was to enable controlling vehicles with reusable booster, which can be configured to not burn all of its fuel (e.g. by having some of the fuel counted as its dry mass).
PEGAS by default would not shut its engines, potentially causing a collision during separation.
Setting this flag to `TRUE` overrides this behavior, causing PEGAS to shutdown the booster's engines before staging.

##### Engines
Key of type `LIST` containing `LEXICON`s.
Each element contains parameters of one engine in a following way:

Key    | Units | Meaning
---    | ---   | ---
isp    | s     | Vacuum specific impulse of the engine
thrust | N     | Vacuum thrust of the engine
*flow* | kg/s  | **Optional key**: instead of specifying `thrust`, one can directly use mass flow

Trick: if your vehicle has multiple engines of the same type, you can go away with a list containing just one element.
The only thing you need to do is input the combined thrust of all engines.
For example, a single engine version of the Centaur upper stage, would look like `LEXICON("isp",422,"thrust",67000)`.
A double engine version can be easily created by writing: `LEXICON("isp",422,"thrust",2*67000)`.

##### Staging
Key of type `LEXICON`.
Defines means of activation of *this* stage, optionally also separation of the *previous* one.

Key                | Type/units | Required?                       | Meaning
---                | ---        | ---                             | ---
jettison           | `boolean`  | always                          | Does the previous stage need to be explicitly separated?
waitBeforeJettison | s          | if `jettison` is `TRUE`         | Wait between the staging sequence start and separation.
ignition           | `boolean`  | always                          | Does the current stage need to be explicitly ignited?
waitBeforeIgnition | s          | if `ignition` is `TRUE`         | Wait between jettison and ignition sequence start.
ullage             | `string`   | if `ignition` is `TRUE`         | Does the current stage need an ullage burn? Allowed values: `"none"`, `"srb"`, `"rcs"`.
ullageBurnDuration | s          | if `ullage` is **not** `"none"` | Wait between ullage sequence start and engine ignition.
postUllageBurn     | s          | if `ullage` is `"rcs"`          | Wait between engine ignition and RCS ullage push disengagement.

Example use-cases:
* For a launch vehicle with a booster-sustainer first stage (eg. Atlas V, Space Shuttle), the active guidance phase activates in the middle of an engine burn; in this case the staging sequence on the first guided stage needs `jettison = FALSE` and `ignition = FALSE`.
* If the vehicle staging list (as in game, in VAB) treats separation and engine ignition as a single event (spacebar hit), `jettison = FALSE` and `ignition = TRUE`.
* If you need one spacebar hit to separate, and another to ignite the engines, use `jettison = TRUE` and `ignition = TRUE`.

### Sequence
`GLOBAL sequence IS LIST().`

Variable of type `LIST` containing `LEXICON`s.
Allows executing custom events.
Each element of the list is a separate event.
It is **crucial** that the order of events in the list is the same as the order of executing them.

Key      | Type/units | Meaning
---      | ---        | ---
time     | s          | Time after liftoff, when this event is to be executed.
type     | `string`   | Type of the event. See below for the complete list.
message  | `string`   | Optional\*. Message that will be printed in the terminal when the event is executed.
throttle | `scalar`   | **Used only if** `type` **is** `"throttle"`. Desired throttle setting, value in range \[0-1\].
massLost | `scalar`   | **Used only if** `type` **is** `"jettison"`. Informs the system of mass amount lost in the process.
angle    | `scalar`   | **Used only if** `type` **is** `"roll"`. New roll angle.

\* - for events of type `throttle` and `roll` message will be automatically generated.

Available event types:

Type     | Short\* | Explanation
---      | ---     | ---
print    | p       | Prints `message` in the GUI, nothing else.
stage    | s       | Hits spacebar (a single `STAGE.` command in kOS).
jettison | j       | Like `stage` but accounts for the mass lost during the event (subtracting the value under `massLost` key).
throttle | t       | Sets the throttle to given value (`throttle` key) - only works during the passive guidance phase.
roll     | r       | Changes the roll component of vehicle attitude (pitch and yaw are dynamically calculated).

\* - can be used instead of the full event type name.

### Mission
`GLOBAL mission IS LEXICON().`

Variable of type `LEXICON`.
Defines the target orbit.
PEGAS allows launch to an orbit specified by selecting a target in the universe map, which allows you to enter only some of the keys.

Key         | Type/units | Meaning
---         | ---        | ---
payload     | kg         | **Optional**. Mass of the payload, that will be added to mass of each guided stage.
apoapsis    | km         | Desired apoapsis above sea level.
periapsis   | km         | Desired periapsis above sea level.
altitude    | km         | **Optional**. Desired cutoff altitude above sea level. If not specified, will be set to periapsis.
inclination | deg        | **Optional**. Inclination of the target orbit. When not given, will be set to launch site latitude (absolute).
LAN         | deg        | **Optional**. Longitude of ascending node of the target orbit. When not given, will be calculated for an instantaneous launch.
direction   | `string`   | **Optional**. Direction of launch. Allowed values: `north`, `south`, `nearest`. By default it is `nearest`.

In case of selecting target from the map, `inclination` and `LAN` will be overwritten from the target data, but apoapsis and periapsis will not.  
Inclination can be omitted - it will be then set to the launch site latitude *magnitude*.
LAN can also be omitted - in this case it will be calculated for a "right now" launch, depending on `direction`.
If both LAN is set free and `direction` is set to `nearest`, the latter will be overridden with `north`.

**Notice**: if the `controls` struct overrides the launch azimuth and `direction` is set to `nearest`,
a conflict may happen where eg. the nearest launch opportunity is southerly but the launch azimuth optimized for a northerly launch.
PEGAS will not try to figure out this conflict but obey, attempting to fly an impossible mission - be careful!

## Communication module

PEGAS can communicate with other CPUs on the same vessel by responding to data requests or executing commands in flight.
This feature uses the [kOS Communication](https://ksp-kos.github.io/KOS/commands/communication.html) system.
All messages need to be sent to the CPU that PEGAS is running on.

Example code for sending a message: `PROCESSOR("cpu_nametag"):CONNECTION:SENDMESSAGE(message).`.

### Message Structure
Each message needs to be a `LEXICON` containing the following keys and values:

Key    | Type      | Meaning
---    | ---       | ---
type   | `string`  | What kind of message is it. Allowed values: `"request"` or `"command"`. PEGAS will always respond with a message type `"response"`.
data   | (depends) | List of requested data or commands to be executed. Format depends on message type, please check the [Requesting data](#requesting-data) and [Sending commands](#sending-commands) sections below for details.
sender | `string`  | **Optional**. If you include the sender field with your CPU nametag, PEGAS will send the response directly to that CPU's message queue, otherwise it will send it to the vessel's queue (See [kOS message queues](http://ksp-kos.github.io/KOS_DOC/commands/communication.html#message-queues) for details).

Example message: `LOCAL message IS LEXICON("type", "request", "data", LIST("data_1", "data_2"), "sender", CORE:TAG).`.

A response from PEGAS will always be in the same format:
* `type` being `"response"`.
* `data` containging a `LEXICON` with keys being command names/requested data names, and values being the data requested, boolean confirmation of command execution or an error message. **NOTE**: If the message sent to PEGAS contained `data` in unrecognised format, the `data` field in response will contain a `string` with an error message, NOT a `LEXICON`.
* `sender` being the nametag of CPU that PEGAS runs on.

An example response: `LEXICON("type", "response", "data", LEXICON("liftoffTime", 3579844, "upfgActivation", 152), "sender", "pegas_cpu_nametag")`.

Please note that all error messages start with `"ERROR"`, followed by a space and the actual message `"(Error occured while creating error message)`.

### Requesting data

PEGAS can provide some information to other CPUs upon request. To request data, your message needs to have a type of `request` and the data should contain one of the following:
* `string` with a name of data being requested (see table below). Example: `"data", "liftoffTime"`.
* `list` of `strings` with names of multiple data being requested. Example: `"data", LIST("liftoffTime", "launchAzimuth")`.

Table of available data:

Available data | Return type | Description
---            | ---         | ---
liftoffTime    | `scalar`    | Time when the rocket is scheduled to launch, provided as seconds since epoch.
launchAzimuth  | `scalar`    | Direction in which the rocket will be heading in passive guidance mode, provided as compass heading.
upfgActivation | `scalar`    | Time (seconds after liftoff) when UPFG guidance is scheduled to start.
upfgConverged  | `boolean`   | Whether the UPFG has already converged or not.

### Sending commands

PEGAS can execute commands requested by other CPUs. To send a command, your message needs to have a type of `command` and the data should contain one of the following:
* `string` containing a command name (`string`). This can only be used to send a single command that doesn't require any parameters. Example: `"data", "setUpfgTime"`,
* `list` containing a single command name (`string`) followed by parameters (types depend on the command, see table below). Example: `"data", LIST("engineShutdown", "engine_1", "engine_2")`,
* `list` of `lists`. Allows for sending multiple commands in a single message. The outer list is a list of commands, and each of the inner lists has to follow the above format (regardless of whether the given command takes parameters or not). Example: `"data", LIST(LIST("setThrottle", 0.7, 0.5), LIST("setUpfgTime"))`.

Table of available commands:

Command        | Parameters            | Availability  | Description
---            | ---                   | ---           | ---
setUpfgTime    | `scalar` (optional)   | passive phase | Change time when the UPFG guidance phase should start. Useful if it can't be predicted before launch. If called without parameters, will default to now (`TIME:SECONDS`).
engineShutdown | multiple `string`s    | passive phase | Shutdown engines specified by nametags. Multiple nametags and multiple engines with the same nametag allowed.
setThrottle    | 2 `scalar`s           | passive phase | Change the throttle setting in flight. Parameters, in that order: desired throttle setting, minimum engine throttle, both as numbers between 0 and 1.
