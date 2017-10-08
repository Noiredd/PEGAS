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
launchAzimuth      | deg   | optional | Overrides automatic launch azimuth calculation, giving some basic optimization capability
initialRoll        | deg   | optional | Angle to which the vehicle will roll during the initial pitchover maneuver (default is 0)

### Vehicle
`GLOBAL vehicle IS LIST().`

Variable of type `LIST` containing `LEXICON`s.
Each element of the list contains physical descriptions of each stage to be guided with UPFG, as well as desciption of the staging sequence.
The following table gives list of all possible keys each element of the list can have, some of which are required, others optional.

Key         | Type/units | Opt/req   | Meaning
---         | ---        | ---       | ---
name        | `string`   | required  | Name of the stage that will be printed in the terminal
massTotal   | kg         | optional* | Total mass of the **entire vehicle** at the moment of ignition of this stage
massFuel    | kg         | optional* | Mass of the fuel in this stage at the moment of ignition
massDry     | kg         | optional* | `massTotal` - `massDry`
gLim        | G          | optional  | Acceleration limit to be imposed on this stage (requires throttling engines)
minThrottle | (0.0-1.0)  | optional**| Minimum possible throttle of this stage's engines (for Realism Overhaul)
throttle    | (0.0-1.0)  | optional  | Nominal throttle for this stage's engines (default = 1.0)
engines     | `list`     | required  | Parameters of each engine in the stage (details further)
staging     | `lexicon`  | required  | Description of method of activation of this stage (details further)

\* - of the three fields, `massTotal`, `massFuel` and `massDry`, one can be skipped, but **two** have to be given.  
\*\* - required if `gLim` is given.

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

Type       | Short\* | Explanation
---        | ---     | ---
print    | `p`     | Prints `message` in the GUI, nothing else.
stage    | `s`     | Hits spacebar (a single `STAGE.` command in kOS).
jettison | `j`     | Like `stage` but accounts for the mass lost during the event (subtracting the value under `massLost` key).
throttle | `t`     | Sets the throttle to given value (`throttle` key) - only works during the passive guidance phase.
roll     | `r`     | Changes the roll component of vehicle attitude (pitch and yaw are dynamically calculated).

\* - can be used instead of the full event type name.

### Mission
`GLOBAL mission IS LEXICON().`

Variable of type `LEXICON`.
Defines the target orbit.
PEGAS allows launch to an orbit specified by selecting a target in the universe map, which allows you to enter only some of the keys.

Key         | Units | Meaning
---         | ---   | ---
payload     | kg    | **Optional**. Mass of the payload, that will be added to mass of each guided stage.
apoapsis    | km    | Desired apoapsis above sea level.
periapsis   | km    | Desired periapsis above sea level.
altitude    | km    | **Optional**. Desired cutoff altitude above sea level. If not specified, will be set to periapsis.
inclination | deg   | Inclination of the target orbit.
LAN         | deg   | Longitude of ascending node of the target orbit.

In case of selecting target from the map, `inclination` and `LAN` will be overwritten from the target data, but apoapsis and periapsis will not.
Only in this situation one can go away with not inputting inclination and LAN.
