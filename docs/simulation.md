## PEGAS MATLAB implementation
This article is meant as an introduction to MATLAB part of **PEGAS** (from this point on just "PEGAS").
The basic concepts of the program are outlined here.
For details, see help (and code) for each function in the repository (`help flightsim3d` etc.).

### Introduction
PEGAS is a spaceflight **launch simulation** environment, in this aspect at least as detailed as Kerbal Space Program (KSP).
In its typical usage scenario (see `testShuttle.m` for example), a launch from an launch site on Earth into some target orbit is simulated.
The code is however flexible, so with a bit more tweaking any kind of on-orbit maneuver is possible (within the rather tight limits of physics - see [physics](#physics) and forget about complete moonshot simulations).

There exist several conceptual differences between a simulation in PEGAS and, for example, a KSP or Orbiter mission.
Primarily, a "mission" in PEGAS is not a continuous flight, in which a vehicle undergoes changes (staging).
Instead, it is divided into small pieces (which will be referred to as "simulations"), in each of which an invariable vehicle flies from some initial condition for some limited period of time under some scheme of control.
By this design, each stage of a vehicle must be flown in an individual simulation, and the final condition of each of them becomes the initial condition for the subsequent.
For this reason, each stage of a vehicle must be a complete, self-sufficient description of a vehicle.
Additionally, a "stage" does not necessarily have to be a physical stage - shift of a control scheme (eg. from an atmosphere-optimized method to precise orbit targetting algorithm) also must be handled by staging.
Another consequence of this setup is that guidance (control) is closely tied with the physical design of the vehicle - control algorithm changes can only be made when the vehicle stages.
So if such a change is required during an active burn, an artificial staging event has to be created (which requires a redefinition of a vehicle itself).

Another crucial difference is a physics setting, which is stripped down to the necessary bones.
At all times, only one body is simulated: the vehicle.
It is subject to three forces: gravity, thrust and atmospheric drag.
Gravity follows the SoI concept of KSP, ie. only the parent body can have gravitational effect on the vehicle, but the actual body does not really exist.
Nor do any other bodies (not even "on rails" like in KSP) - effectively any interactions like crashes or SoI changes cannot be simulated.

Thrust follows several possible logic paths (engines can be throttlable or not, thrust can be constant or adjusted for acceleration limit, or even follow a custom function).
Any changes in thrust mode are to be simulated by staging: for example engaging a 3G acceleration limit in a Space Shuttle (occured before SSME MECO) is a new logical stage.

### Flow
Most generally, the program starts with initialization (creation of the environment, vehicle, setting up initial and target conditions), then *a series* of simulations, finally the visualization of results.
Basic scenario assumes a multistage vehicle being launched into some target *condition* (usually but not necessarily an orbit).
In this setup, the first stage is flown with a separate control scheme, ie. either a pitch program (if available) or naive, gravity turn (pitch over and hold prograde). Further stages are guided with [Unified Powered Flight Guidance](upfg.md) onto a given target, until that target is reached or vehicle runs out of fuel.

The smallest unit is a single simulation, handled by single call to `flightSim3D`, corresponding to a single physical or logical stage of a vehicle.
In a "basic scenario" a `flightManager` function calls it for each stage, automatically handling the final/initial conditions conversion.
Stages, as already mentioned, correspond to different states of the vehicle, be it physical (jettisoning high-thrust atmospheric engine in favor of a vacuum-optimized engine, dropping strap-on boosters) or logical (switching from constant thrust to constant acceleration mode, changing atmospheric guidance algorithm to orbital targetting).
Minor events - like fairing separation, jettison of an interstage ring or a launch abort tower - can be simulated as stages, but there exists a hack-ish but simpler way to handle them (see `help flightSim3D`, section "INPUT", entry "jettison").

A simulation requires 3 components:
* vehicle definition,
* initial conditions,
* method of control.

Vehicle is in general an array of structures corresponding to successive stages.
Each stage must be a logically self-sufficient vehicle.
For a 3-stage example: stage 1 will be a complete vehicle as standing on the launch pad, stage 2 will be a vehicle state after jettisoning the first stage (both stages 2 and 3), stage 3 is the final stage alone.

Initial conditions is a structure defining a vehicle state at the beginning of simulation.
There are two possible versions: stationary condition on a launch pad, defined by latitude and longitude of the site, and in-flight condition, defined by position and velocity vectors.
`flightManager` makes use of `resultsToInit` to extract final vehicle state and convert it into an initial conditions for the next stage.

Method of control is a structure that defines the way a vehicle is steered during the flight. There are 4 main options:
* naive gravity turn - this simulates the simplest and safest approach to atmospheric flight, ie. pitching over at a given time by a given angle and holding prograde until MECO (allows choosing a constant launch azimuth),
* preprogrammed pitch - vehicle is passively guided using a pitch vs time table with a launch azimuth held constant - this is similar to how Saturn V realized its first stage flight (note that there is no tool to create this pitch program - so use at your own risk),
* [Unified Powered Flight Guidance](upfg.md) - general guidance algorithm as used on the actual Space Shuttle GN&C computer - this requires a definition of a target orbit (generated by `launchTargeting`) and a complete knowledge of the vehicle (provided automatically by `flightSim3D`) and actively, iteratively generates pitch and yaw commands to guide the vehicle onto the target,
* coast phase - this setting disables thrust (and mass flow), making the vehicle freely coast for a specified period of time.

For most of those required structures there exist generator functions that create compatible `struct`s (see end of this document).

### Physics
PEGAS is a 3 degrees of freedom (3DoF) simulation, ie. vehicles are modelled as point masses with certain dynamics.
Frame of reference is stationary, its center coincident with the center of the body the vehicle is launched from (it can be easily configured to be anything else than Earth).
The Earth's rotation is partially implemented, ie. the vehicle does receive the initial velocity from this rotation, but the rotation of the actual body is not simulated (because the body does not actually exist).
Since the simulation keeps track on both absolute and surface-relative velocity, this does not affect the dynamics of the vehicle (i.e. atmospheric velocity is properly handled).
However, since Earth never rotates, the vehicle's ground track will not be entirely realistic.
Additionally, longitude of ascending node is affected: attainable LAN heavily depends on the choice of launch site (in real world or KSP also on time-of-launch).
This is however countered by simulating a time-of-launch adjustment as a difference between the target LAN and LAN of the orbit passing right over the launch site is calculated (`help launchTargeting`).

Gravity model is identical with KSP - there is only one body directly acting on the vehicle.
But in fact, there are no other bodies simulated at all (aside from the vehicle), so more complex missions are not possible without excessive changes.
Gravity is a simple, Newtonian inverse-square model with no account for Earth oblateness (body is only described by its radius and gravity parameter).
Atmosphere (air pressure and temperature) is derived from Realism Overhaul data files, sampled as a function of altitude.
Air density is calculated from the ideal gas law.

Vehicle engines are simulated similarly to ones in KSP, ie. they are defined by mass flow and Isp (surface level and vacuum), although the technicalities behind them are vastly different (read the next section).
Isp changes with atmospheric pressure, creating a realistic thrust behavior.
Solid rocket motors with different thrust dynamics are also possible.

### Vehicle
Vehicle is an array of structures corresponding to stages.
Each stage is modelled as a self-sufficient, independent "logical vehicle".
Parameters of separate stages are not interdependent - this makes it technically possible to launch an entire Saturn V on top of a WAC Corporal (it is up to the user to define their vehicles realistically).
Basic layout of a stage struct is as follows:

Field   | Unit     | Meaning
---     | ---      | ---
MODE    | (int)    | Thrust mode: **1** - constant thrust, **2** - constant acceleration.
m0      | kg       | Initial mass of the vehicle.
maxT    | s        | Maximum time of burn of this stage.
gLim    | G        | Acceleration limit. *Ignored if MODE==1.*
area    | m^2      | Area of active cross-section for drag calculation.
drag    | m/2, -   | Drag coefficient `Cd` as a function of velocity, given by (n,2) array: velocity in first column, drag coefficient corresponding to this velocity in second column.
engines | (struct) | Array of structures of type engine, defining all actively thrusting engines in this stage.

One thing to notice in this description is lack of notion of "fuel".
Vehicle does indeed burn its mass, but instead of limiting its capabilities by amount of fuel, it needs to be converted directly into time.
There exists a function to easily do this (and other conversions of this kind), see `help vehicleTools`.

Another limitation concerns engines.
The structure is organized in the following way:

Field | Unit   | Meaning
---   | ---    | ---
mode  | (int)  | Throttle mode: **1** - free throttle, **2** - throttle program.
isp0  | s      | Vacuum specific impulse (Isp).
isp1  | s      | Surface level specific impulse (Isp).
flow  | kg/s   | Nominal mass flow rate at 100% throttle.
data  | -      | If **mode==1**: throttle limits as an array of length 2 (minimum throttle as first element, maximum as second), given as fraction of nominal throttle (ie. value of 1.0 corresponds to 100% thrust).
data  | s, -   | If **mode==2**: throttle as a function of time, given by (n,2) array: time in the first column, throttle setting corresponding to this time in the second column (as fraction of nominal, see above).

*Mode* distinguishes between two engine types:
* *free throttle*, where the **program is in control** of the throttle (example engine types that would be simulated this way: liquid fuel engines in constant acceleration mode, non-throttleable engines)
* *throttle program*, in which throttle setting is a **predetermined function** of time (specifically designed for solid rocket motors, but can fulfill a variety of other needs, for instance preprogrammed reduction of thrust during max-q).

*Data* means different things depending on chosen mode and should be declared with care.

There exists a relation between engine::mode and stage::MODE.
If a vehicle is in a constant thrust mode, each of its free-throttled engines is being run at 100% throttle (nominal mass flow), and each of its programmed-throttle engines follows its program.
*This ignores throttle limits, so if for some reason an engine has both throttle limits above or below 1.0 (ie. prohibiting a 100% thrust), these will have no effect.*
In a constant-acceleration mode, an engine will be throttled to abide by the acceleration limit (this *will* obey the throttle limits, so if a smaller than possible thrust is requested, the engined will only throttle so low, effectively ignoring the acceleration limit).
However, due to the simplicity of a function that calculates vehicle thrust (`getThrust`), a constant-acceleration stage must only have one single engine.
For this reason, if multiple engines of the same type are to be used, they should be collapsed into a single engine with proportionally larger nominal mass flow rate.
Naturally, this engine must not be following a throttle program (ie. has to be a free-throttled engine).

### Structs and flags
This section lists layouts of all structures used by PEGAS.
Attention must be paid that all structures meant to be stacked into arrays (eg. vehicle, engine) have the exact same set of fields.
*Origin* stands for center of the reference frame (center of the Earth).


#### Initial
Initial physical state of the vehicle.

Field    | Unit  | Meaning
---      | ---   | ---
type     | (int) | Type of initialization: **0** - launch site, **1** - vehicle continues motion
lon      | deg   | Longitude. *Ignored if type == 1.*
lat      | deg   | Latitude. *Ignored if type == 1.*
alt      | m     | Altitude above sea level. *Ignored if type == 1.*
t        | s     | Mission elapsed time. *Ignored if type == 0.*
r        | m     | Vehicle position relative to origin (XYZ vector). *Ignored if type == 0.*
v        | m/s   | Vehicle velocity (XYZ vector). *Ignored if type == 0.*
Generators:  
`createLaunchSite` - creates a type 0 initial struct (launch site),  
`resultsToInit` - converts a [results](#results) struct to a type 1 initial struct.  
Typical functions that use it:  
`flightSim3D`.

#### Target
Desired insertion conditions for UPFG.

Field    | Unit | Meaning
---      | ---  | ---
radius   | m    | Altitude from origin
velocity | m/s  | Velocity vector magnitude
angle    | deg  | Flight path angle measured from the horizon
normal   | -    | Unit vector (XYZ) normal and negative\* to the desired orbital plane

\* - *for a prograde equatorial orbit this vector should point south, that's how UPFG was written*.

Generators:  
`launchTargeting`.  
Typical functions that use it:  
`flightSim3D` - only passes it down to `unifiedPoweredFlightGuidance`.

#### Control
Method of controlling a vehicle stage.

Field    | Unit   | Meaning
---      | ---    | ---
type     | (int)  | Type of control: **0** - pitch over and hold prograde, **1** - programmed pitch, **2** - obsolete, **3** - UPFG, **5** - uncontrolled coast phase
pitch    | deg    | Initial pitchover angle.\* *Ignored if type != 0.*
velocity | m/s    | Pitchover velocity.\* *Ignored if type != 0.*
azimuth  | deg    | Launch azimuth (measured from east to north, counterclockwise). *Ignored if type > 1.*
program  | s, deg | Pitch program as a function of time, given by (n,2) array: time in the first column, pitch angle corresponding to this time in the second column. *Ignored if type != 1.*
target   | struct | Struct of type target. *Ignored if type != 3.*
major    | s      | Length of the UPFG cycle (ie. UPFG routine is called every this often). *Ignored if type != 3.*
length   | s      | Length of the coast phase. *Ignored if type != 5.*

\* - *will go straight up until reaches given velocity, then starts pitching over 1 degree/s towards the given azimuth; when reaches the given pitchover angle will hold waiting for the AoA to decrease to zero; then will hold prograde until MECO.*

Generators:  
`flightManager` - will create *coast* (type 5) and *UPFG* (type 3) structs. For now the users has to create type 0 or 1 structs.  
Typical functions that use it:  
`flightSim3D` - one of primary requirements.

#### State
Current vehicle physical state for UPFG.

Field    | Unit | Meaning
---      | ---  | ---
time     | s    | Mission elapsed time
mass     | kg   | Total vehicle mass
radius   | m    | Vehicle position relative to origin (XYZ vector)
velocity | m/s  | Vehicle velocity (XYZ vector)

Only `unifiedPoweredFlightGuidance` uses it, while `flightSim3D` creates it when necessary.

#### UPFG internal
Current internal state of the UPFG algorithm.

Field    | Unit   | Meaning
---      | ---    | ---
cser     | struct | Struct of type CSER (defined below)
rbias    | m      | XYZ vector, see definition of `Rbias` in [upfg.md](upfg.md)
rd       | m      | XYZ vector, see definition of `Rd` in [upfg.md](upfg.md)
rgrav    | m      | XYZ vector, see definition of `Rgrav` in [upfg.md](upfg.md)
tb       | s      | Elapsed time of this burn phase
time     | s      | Mission elapsed time
tgo      | s      | Calculated time-to-go
v        | m/s    | Vehicle velocity (XYZ vector)
vgo      | m/s    | Calculated velocity-to-go, see [upfg.md](upfg.md)

Only `unifiedPoweredFlightGuidance` uses and returns it, but `flightSim3D` creates it for the first time.

#### CSER
Current logical state of the CSE routine, see [CSEroutine.m](../MATLAB/CSEroutine.m).

Field    | Unit   | Meaning
---      | ---    | ---
dtcp     | s      | Converged value of transfer time interval
xcp      | -      | Converged value of *x* corresponding to the time interval
A        | -      | `Un` function parameter
D        | -      | `Un` function parameter
E        | -      | `Un` function parameter

Only `CSEroutine` uses and returns it, it is a part of *UPFG internal* struct and as such it comes through `unifiedPoweredFlightGuidance`, and `flightSim3D` creates it for the first time along with it.

#### Guidance
Guidance parameters calculated by UPFG.

Field    | Unit   | Meaning
---      | ---    | ---
pitch    | deg    | Pitch angle measured from the local *up* direction (0 = straight up)
pitchdot | deg/s  | Rate of change of pitch angle
yaw      | deg    | Yaw angle measured from the local *east* direction towards local *north* (0 = straight east, -90 = towards the south pole)
yawdot   | deg/s  | Rate of change of yaw angle
tgo      | s      | Time-to-go (cutoff)

Generators:  
`unifiedPoweredFlightGuidance` - primary output of the function.  
Typical functions that use it:  
`flightSim3D` - uses it internally to guide the vehicle during the simulation.

#### Results
Nested structure returned by `flightSim3D`. Fields in the structure root as well as *Orbit* field contain simulation summary (final conditions). *Plots* contain complete logs of simulation variables as functions of simulation time (iteration).

Field        | Subfield | Unit | Meaning
---          | ---      | ---  | ---
Altitude     | -        | km   | Altitude above sea level.
Apoapsis     | -        | km   | Orbit apoapsis above sea level.
Periapsis    | -        | km   | Orbit periapsis above sea level.
Velocity     | -        | m/s  | Velocity magnitude.
VelocityY    | -        | m/s  | Vertical velocity component magnitude.
VelocityT    | -        | m/s  | Tangential velocity component magnitude.
maxQv        | -        | Pa   | Maximum dynamic pressure encountered.
maxQt        | -        | s    | Time at which maximum dynamic pressure was encountered.
LostGravity  | -        | m/s  | Velocity lost due to gravity. Makes most sense for stages flying straight up - for upper stages making long burn arcs this might get a little weird.
LostDrag     | -        | m/s  | Velocity lost due to atmospheric drag.
LostTotal    | -        | m/s  | Combined velocity losses.
BurnTimeLeft | -        | s    | Fuel left in this stage's tank after cut off, measured by remaining burn time. Larger number for the same mission = better efficiency.
Orbit        | SMA      | m    | Semi-major axis of the orbit (due to the math behind this, this and other orbital parameters will be calculated with no regard to whether the vehicle actually is in orbit or not.)
.            | ECC      | -    | Eccentricity.
.            | INC      | deg  | Inclination.
.            | LAN      | deg  | Longitude of ascending node.
.            | AOP      | deg  | Argument of periapsis.
.            | TAN      | deg  | True anomaly.
Plots        | t        | s    | Mission elapsed time in each simulation step, (n,1) array.
.            | r        | m    | Vehicle position in each simulation step, (n,3) array.
.            | rmag     | m    | Vehicle position magnitude (distance from origin), (n,1) array.
.            | v        | m/s  | Vehicle velocity, (n,3) array.
.            | vy       | m/s  | Vehicle vertical velocity magnitude, (n,1) array.
.            | vt       | m/s  | Vehicle tangential velocity magnitude, (n,1) array.
.            | vmag     | m/s  | Vehicle total velocity magnitude, (n,1) array.
.            | F        | N    | Vehicle thrust magnitude, (n,1) array.
.            | a        | m/s^2| Vehicle acceleration magnitude, (n,1) array.
.            | q        | Pa   | Dynamic pressure, (n,1) array.
.            | pitch    | deg  | Pitch command log, (n,1) array.
.            | yaw      | deg  | Yaw command log, (n,1) array.
.            | vair     | m/s  | Vehicle surface-relative velocity, (n,3) array.
.            | vairmag  | m/s  | Vehicle surface-relative velocity magni tude, (n,1) array.
.            | angle_ps | deg  | Surface-relative flight path angle (angle to "up").
.            | angle_ys | deg  | Surface-relative flight "yaw" (angle from "east" as measured towards "north").
.            | angle_po | deg  | Absolute (orbital) flight path angle.
.            | angle_yo | deg  | Absolute (orbital) flight "yaw" angle.
.            | DEBUG    | (struct) | Debug data from UPFG calls. Too long to document exactly.
ENG          | -        | -    | Flag informing of state of the engine at end of stage: **-1** - most likely error, **0** - engine fuel deprived, **1** - engine still running, **2** - cutoff as scheduled by UPFG, **3** - emergency cutoff (UPFG most likely went crazy).

Generators:  
`flightSim3D` - returns this normally.  
Typical functions that use it:  
`telemetry`, `trajectory` - use it indirectly.

#### Flight
Packed flight results of multiple stages.

Field   | Type           | Meaning
---     | ---            | ---
powered | results struct | Results of all powered phases in order.
coast   | results struct | Results of all coast phases in order. Information on order of coast phases in relation to powered phases is not directly available (.Plots.t encodes it).
n       | int            | Index of the last powered phase.

Generators:  
`flightManager` - returns this normally.  
Typical functions that use it:  
`telemetry`, `trajectory` - ingest this struct directly, extracting *results* struct within themselves.
