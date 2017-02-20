## PEGAS MATLAB implementation
This article is meant as an introduction to MATLAB part of **PEGAS**.

### Introduction
Initialization, vehicle definition, target specification, simulation, visualization.

### Physics
3dof, stationary frame, stationary Earth (LAN consequences), simplified atmosphere

### vehicle
each stage is a self-sufficient, independent vehicle

### Initial and target conditions

### Structs and flags
"Origin" stands for center of the reference frame (center of the Earth).

**Target** - desired insertion conditions for UPFG.

Field    | Unit | Meaning
---      | ---  | ---
radius   | m    | Altitude from origin
velocity | m/s  | Velocity vector magnitude
angle    | deg  | Flight path angle measured from the horizon
normal   | -    | Unit vector (XYZ) normal and negative\* to the desired orbital plane

\* - *for a prograde equatorial orbit this vector should point south, that's how UPFG was written*.

**State** - current vehicle physical state.

Field    | Unit | Meaning
---      | ---  | ---
time     | s    | Mission elapsed time
mass     | kg   | Total vehicle mass
radius   | m    | Vehicle position relative to origin (XYZ vector)
velocity | m/s  | Vehicle velocity (XYZ vector)

**UPFG internal** - current logical state of the UPFG algorithm

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

**CSER** - current logical state of the CSE routine, see [CSEroutine.m](CSEroutine.m).

Field    | Unit   | Meaning
---      | ---    | ---
dtcp     | s      | Converged value of transfer time interval
xcp      | -      | Converged value of *x* corresponding to the time interval
A        | -      | `Un` function parameter
D        | -      | `Un` function parameter
E        | -      | `Un` function parameter

**Guidance** - calculated guidance parameters.

Field    | Unit   | Meaning
---      | ---    | ---
pitch    | deg    | Pitch angle measured from the local *up* direction (0 = straight up)
pitchdot | deg/s  | Rate of change of pitch angle
yaw      | deg    | Yaw angle measured from the local *east* direction towards local *north* (0 = straight east, -90 = towards the south pole)
yawdot   | deg/s  | Rate of change of yaw angle
tgo      | s      | Time-to-go (cutoff)

initial, control, vehicle, results

### Code summary
?
