## PEGAS launch tutorial
The bare essence of what you need to do to simulate your launch.
This assumes you know what kind of mission are you going to fly (where from, where to) and you collected all parameters of the vehicle you are going to use.
We will be assuming a Space Shuttle mission from KSC to a parking orbit in ISS plane.
The atmospheric part of ascent will follow a *pitch over and hold prograde* mode, switching to UPFG upon jettisoning the SRBs.
Since the mission is manned, the acceleration will be limited to 3G.
This creates 3 stages: SSME+SRB atmospheric ascent, SSME full thrust mode, SSME in constant acceleration mode.

### Required information
* Launch site. `createLaunchSite` offers some limited choice of ready-to-use sites, but if yours is not there, you will need to know its **latitude**, **longitude** and **altitude ASL**.
* Target orbit, particularly **apoapsis**, **periapsis** and **inclination**. If you needed to target LAN too, `launchTargeting` will help you calculate when do you need to launch (see `help` on it, read on "lan" returned value).
* Vehicle information, stage by stage. For each you will need to find/calculate its **initial mass**, **maximum burn time** (time equivalent of fuel in the tanks), **cross-section area**, **drag coefficient** as function of velocity, and the following about each of its engines: **sea level** and **vacuum Isp**, nominal **mass flow**, **throttle limits** ([1.0 1.0] for non-throttleable engines) and for specialized applications (SRB) also **thrust profile**.  
See `SpaceShuttleThrustProfile` for an example SRB thrust profile.

### Overview
There are a few steps you typically go through when launching a vehicle.
File `testShuttle.m` is an example of such typical launch, on the example of a Space Shuttle mission.
To illustrate those typical steps, it is structured into 8 sections:
* initialization: `initSimulation` creates the world, setting up global variables like Earth radius, its rotation period, atmosphere etc.,
* construction of the vehicle (`SpaceShuttle`),
* launch site definition by loading a predefined site using `createLaunchSite`,
* target orbit definition and creation of a UPFG target,
* atmospheric guidance definition (`stage1`, in this example it is a simple pitch-over-and-hold-prograde),
* launch using `flightManager`,
* results visualization,
* clearing the environment from temporary variables.

### Creating the vehicle
File `SpaceShuttle.m` contains a stage-by-stage definition of a Space Shuttle Endeavour.
It starts with some technical bits: first it removes an existing vehicle (but you are free to have several vehicles in your workspace under different names), then sets variables corresponding to the mass of the orbiter itself and payload.
Finally it loads a reference SRB thrust profile from `SpaceShuttleThrustProfile.m`.
Then each stage is defined separately in the following order:
* launch pad configuration (Orbiter with payload, External Tank, two Solid Rocket Boosters),
* Orbiter with payload + External Tank, main engines in full thrust mode,
* Orbiter with payload + External Tank, acceleration-limited mode,
* Orbiter with payload only, OMS engines in constant thrust.

Let's review it stage by stage, and, since each is broken up into lines for clarity, line by line. First, the complete launch stack:  
`stage_m0 = 2*587000 + 765000 + orbiter + payload;` simply adds together total masses of all individual components  
`stage_engines(1) = struct(`... creates an array of engines, initializing its first element - the 3 SSMEs; see [struct specification](simulation.md#vehicle) for details  
`stage_engines(2) = struct(`... adds another engine, this is 2 SRBs (notice how its thrust profile is passed in `data` field)  
`stage_time = 124;`  defines for how long can this stage burn  
`stage_area = 2*10.8 + 55.4 + 15.2;` is an estimate of the stack's cross-section area for aerodynamic drag calculation  
`stage_drag = [`... is a VERY rough estimate (to the point of being unrealistic) of the Shuttle's drag coefficient Cd (as function of velocity) - essentially, we want a little drag at first, then some more in the transonic region (`343  1.20;`), and then less and less (similarly to how FAR analysis curves look like)  
`stage = struct(`... combines all the variables we just set into a struct, as per [specification](simulation.md#vehicle); we set `MODE` to 1 for constant thrust, so the `gLim` setting does not matter  
`vehicle(1) = stage;` packs the struct into an array, probably could be done within one line like with the engines.

The next stage begins after the SRB jettison.
Its initial mass is therefore smaller by the mass of SRBs and *all the fuel the SSMEs consumed by now*.
First, let's bring the initial mass of all the components, just without the SRBs:  
`stage_m0 = 765000 + orbiter + payload;`  
We will use a neat tool to get the fuel mass decrement, the `vehicleTools`, but first we need to set up some variables.
Since `vehicleTools` requires a configured stage struct to work, we need to give it one, but *bearing in mind that we are only interested in the SSME fuel consumption*.  
`stage.engines(2) = [];` gets rid of SRBs in our existing (first) `stage` so that we can use `vehicleTools`  
`stage_engines(2) = [];` does the same with the engines struct that we will put in the new stage (second)  
`stage_m0 = stage_m0 - vehicleTools('mass', stage, 1, stage.maxT);` deduces mass of fuel burned by the SSMEs (do `help vehicleTools` to see how it works)  
`stage_time = 320;` is a somewhat magical piece of code - it sets the stage burn time so that at the end of this stage the acceleration at SSME full thrust will be equal to 3G; it could of course be calculated in the following steps:
* use `getThrust` to find the nominal thrust of the stage
* rearrange the formula `force = mass * acceleration` to find mass of the vehicle, at which this thrust will produce your desired acceleration
* subtract this mass from current `stage_m0` to find how much fuel the vehicle needs to burn until that happens
* use `vehicleTools` to find how long will it take to burn that much fuel

The result comes pretty close to 320 seconds.  
`stage_area = 55.4 + 15.2;` recalculates the cross-section area without the SRBs  
`stage = struct(`... combines all those into a new struct  
`vehicle(2) = stage;` packs them into the array.

Next stage begins when the vehicle switches into a constant-acceleration mode with a 3G limit.
We burned some more fuel, so the first thing we do is deduce our initial mass by that:  
`stage_m0 = stage_m0 - vehicleTools('mass', stage, 1, stage.maxT);`  
Then we need to find out how much more fuel do we have left.
This was not mentioned anywhere earlier, but the Shuttle begins its flight with about 730 tons of hydrogen and oxygen in the ET:  
`stage_fuel = 730000 - vehicleTools('mass', stage, 1, 124+320);` since we know how long did the previous stages last, we simply calculate how much fuel was burned during that time  
`stage = struct(`... now we pack the values, largely basing on the previously compiled stage, into a new struct; we set `MODE` to 2, meaning constant-acceleration, and `gLim` to 3 for a functional acceleration limit.  
Now we can call `vehicleTools` again, this time to calculate for how long exactly can this stage burn:  
`stage.maxT = vehicleTools('tgo', stage, 2, [stage_fuel stage.gLim]);`  
We needed to have a functional stage struct in order to do that, now we just updated it with one last value (`maxT`).

Last stage should be self-explanatory at this point.
The only bits worth noticing is the inline calculation of engine mass flow (`'flow', 2*26700/(313*g0),...`) and stage burn time calculation deferred again until after the struct was built.
Line `vehicle(4) = stage;` ends creation of the vehicle structure.
Nothing is returned since the script works directly on the workspace - so at this point the vehicle is ready to run.
The last two lines remove all the temporary variables, **along with the** `payload` **variable** that we might have supplied before the script was ran.
This is on purpose, but might cause some confusion.

### Defining the target
Since we want to use UPFG for vehicle control, we need to supply it with a proper [target structure](simulation.md#target).
The `launchTargeting` function requires a launch site definition and the following 4 values:
* periapsis, or actually the final altitude - in kilometers above sea level,
* apoapsis, or actually the opposing apsis - the same,
* inclination - in degrees,
* angle to launch - because the simulation physics are how they are, the launch always targets an orbit passing directly over the launch site *minus this amount* (see `help launchTargeting` for details).

This function returns LAN of the targeted orbit (if you want to control it, simply imagine adjusting the time of launch), launch azimuth for first stage guidance and the UPFG target struct.

### Running the simulation
Now that we have a vehicle, launch site, target orbit and the first stage guidance, and the simulation is initialized, we can proceed with the launch.
This is as simple as calling `flightManager` with said arguments.
Here we can control a variety of other things, such as simulation precision (here in time steps of 0.2s), time between UPFG cycles (2 seconds) and coast phase lengths (here no coast phases but see `help flightManager` how it should be done).
Additionally we can control minor staging events that we did not put into the vehicle definition.
Those only allow simple mass deductions at specified time, without any other impact on the vehicle - see `help flightSim3D` for details.

This function returns [packed simulation results](simulation.md#flight), distinguishing between powered flight and coast phases.
For each phase there is a separate struct, containing its result summary as well as history of the simulation in form of a time series.
Detailed list of all elements of this struct can be found [here](simulation.md#results).

### Visualizing the results
Those results can be now fed to one of three functions for display.  
`telemetry()` draws plots of simulation history in the form of a following table:

pitch & yaw steering | altitude | vertical velocity
:---: | :---: | :---:
**acceleration** | **downrange distance** | **horizontal velocity**

`trajectory()` draws a 3D plot of the vehicle's powered trajectory.
It has options for rendering an Earth under the plot, as well as some reference vectors. Best to just see how it works and then read its help.  
Finally there is `dbgIntegrals()` - this is only a debug function for UPFG and unless the algorithm malfunctions you will not need it (if your pitch&yaw plots look weird or your vehicle crashes, it's most likely it).
