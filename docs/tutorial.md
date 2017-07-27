## PEGAS tutorial

### Introduction
PEGAS is an ascent autopilot which aims to support a wide variety of vehicles in a wide variety of missions.
It uses the real-world Unified Powered Flight Guidance algorithm for precise and fuel-optimal orbital guidance.
You can read more about the algorithm [here](https://github.com/Noiredd/PEGAS-MATLAB/blob/master/docs/upfg.md), but for a crash course you only need to know the following:
 * UPFG works best outside atmosphere (that is, it is oblivious to atmosphere at all),
 * UPFG requires complete knowledge of the vehicle it is to guide (stage by stage),
 * UPFG does not know the concept of *coasting* (the shorter the pauses between stages, the better),
 * UPFG is not magical and can't do the impossible (direct equatorial orbit from KSC, direct launch to a high orbit, etc.).

The ascent with PEGAS is divided into 3 phases.
In the **pre-launch** phase, the launch time and azimuth are calculated, and the vehicle sits on the pad patiently waiting.
In the **atmospheric ascent** phase, the vehicle goes up for some time, then pitches over by some angle, and once the velocity vector matches its attitude, it keeps prograde (a.k.a. performs a gravity turn).
Finally, in the **closed-loop guidance** phase (or *active guidance*), the UPFG algorithm controls the vehicle, automatically guiding it onto a desired trajectory.  
All of those phases require information from *you*. You need to input the parameters of the gravity turn, information about your vehicle and its intended destination.
In this short tutorial, I will explain how to provide that information.
Reading is strongly encouraged, as PEGAS can be quite complex.

PS: Basic knowledge of kOS syntax is recommended - if you find yourself struggling to understand what's going on, do [read those](http://ksp-kos.github.io/KOS_DOC/language.html).

### Basics
When you `run pegas.`, it is assumed that all the information and the vehicle is ready to go, and the mission is to begin.
Before you do that, you must create 4 structures (as global variables):
 * `vehicle` - contains information about your vehicle, stage by stage,
 * `controls` - defines parameters of the launch, the gravity turn and the moment of phase change,
 * `sequence` - lists all the special events that will happen during the flight (such as payload fairing jettison),
 * `mission` - your desired orbit.

PEGAS will do the following things for you:
 * automatically calculate the launch time and azimuth (phase "0", pre-launch),
 * execute the gravity turn according to your input (phase 1, atmospheric ascent),
 * automatically separate and ignite main stages (phase 2, active guidance),
 * execute all other events you specified in `sequence` (all phases).

To understand everything that follows, a strong emphasis has to be put on the difference between phase 1 and 2.
During the atmospheric ascent, PEGAS does absolutely nothing beyond pitching over and holding prograde.
Only during the active guidance phase it reveals its smarts - not only it dynamically calculates pitch & yaw angles, but it also *automatically handles staging*.  
The reason is that the moment of switching between phases is customizable - so manually recalculating each and every event needed to perform a successful staging (jettison, ullage burn, ignition, ...) would be a terrible hassle.
Instead, for each stage you define *how* to activate it, and when the time comes, PEGAS just does what it's told to.
If you want something to happen during the atmospheric ascent, you need to manually specify it using `sequence`.

In the following 4 sections I will describe the use and meaning of each of the 4 variables you have to create.
Headers are links to reference, be sure to check those out too if in doubt.

### [Controls](reference.md#controls)

This variable controls behavior of the vehicle prior to and during the atmospheric ascent.  
I specifically mention it first because it is crucial to successful flight, and the most difficult one to get right.
As summarized in the reference, it holds 4 keys. Their meaning is as follows:

##### launchTimeAdvance
UPFG allows targeting specific orbital *planes*.
Obviously, launch timing is crucial in order to hit the right plane - intuitively, we know that the launch must occur near the moment that the launch site rotates under the target orbit.
We cannot launch directly when it happens though - the vehicle inherits the Earth's circumferential velocity and continues drifting east, past the orbit.
When UPFG kicks in, it tries to correct for that drift by aiming more towards west than would otherwise be necessary.
Sometimes it can recover from that, other times not - but the more *late* the launch was, the more performance degradation this causes; eventually, vehicle might not have enough fuel to make it to orbit.
Similar situation occurs if the launch was timed too early - UPFG points the vehicle more to the east, to make it *catch up* with the target plane, wasting fuel in the same manner.  
This is a difficult parameter to get right, but fortunately as long as you're in a reasonable range, it only hinders performance (instead of causing critical failures).
Experimentally, I found times of 120-150 seconds to work good.
Vehicles that reach orbit faster will need less advance time, while those long burning upper stages might use more.

##### verticalAscentTime and pitchOverAngle
Atmospheric ascent is difficult from a guidance standpoint, and doing it automatically is far beyond the scope of this project.
Instead, you have control over it using two parameters.
The process is simple: after a certain *time* after liftoff, your vehicle will pitch over by a certain *angle*.
It will hold this attitude until the velocity vector matches it, and then it will lock to prograde, so as to minimize the angle of attack (and hence risk of a RUD).  
Rules are that the earlier you pitch over and the larger its angle, the shallower your ascent.
In a worst case, your vehicle will fall back to the ocean (or get torn apart due to flying too fast too low).
On the other hand, if you pitch over too little and too late, your *gravity turn* will not turn you enough, and your vehicle will be going too high.
Additionally, the larger TWR your vehicle has, the sooner and more aggressively you can turn - a good summary on that is given by [ferram in the RO wiki](https://github.com/KSP-RO/RealismOverhaul/wiki/Ferram-on-Ascent-Profile-and-TWR).  
This is a double important parameter, since it directly influences UPFG phase.
If your desired orbit is circular at, say, 200 km, you want to have about 150-200km apoapsis when the algorithm engages.
Being on 45 km with a 55 km apoapsis, or on 120 km with 350 km apoapsis is a good hint that you did something very wrong during ascent.
UPFG may or may not recover from that.  
Getting `verticalAscentTime` and `pitchOverAngle` right might take several attempts, and might prove particularly difficult if you're getting low ingame FPS during liftoff, or your launch clamps misbehave.
Unpredictable separation that disrupts your vehicle from flying straight can even make a good settings randomly fail. Beware.

##### upfgActivation
This is when the atmospheric ascent ends, and active guidance begins.
You want to be outside the atmosphere when that happens, 40-50 km is good.
From this point on the UPFG is in control over your vehicle, and if you're too low and it decides to pitch up too much, you might experience your vehicle tumbling out of control for a moment, or even rapidly disassembling.

### Sequence

### Vehicle

### Mission
