## Sequence magic

### Why jettisons and shutdowns are so tricky?
We could start by asking the question "why are `sequence` and `vehicle` different things".
The answer to that consists of two statements:
* UPFG needs to know the precise physical description of the entire vehicle (wet/dry masses, engine parameters, etc.) for each stage *prior* to the activation of the entire algorithm;
* the set of things that UPFG wants to know and the set of things that PEGAS needs to know in order to execute the launch (ignition that many seconds prior to liftoff, fairings jettison that many seconds after, etc.) *do not fully overlap*.

This is the reason behind the original concept of PEGAS: physical properties are defined once in one place, and details (when to throttle down for max-q, when to jettison the fairings, etc.) are elsewhere.
The fundamental assumption was that these details ("events") will not influence the physical vehicle significantly, and thus UPFG does not need to know about them.
This assumption was quickly found to not hold in practice.
For example, if during the flight we jettison a payload fairing, that means the dry mass of that stage (and possibly all subsequent stages) is now different than what's stated in the `vehicle` structure.
This means the accelerations that UPFG calculates are off, and with them all of the other values, and with it the entire guidance (if it even converges).

Previously, we dealt with this by performing a small trick during the jettison event:
upon executing a jettison, a given mass (`massLost`) would be subtracted from the mass of the currently activated stage (and possibly subsequent stages, according to a somewhat elaborate logic).
This ensured that the currently flown stage was not immediately broken.
Still, that meant a sudden, unplanned change in vehicle parameters was encountered during active guidance - guidance that was calculated for a *somewhat* different vehicle.
As a result, a slight "jerk" could occur, as UPFG reconverged for the "new" vehicle.

While this approach works for jettison events, it almost certainly wouldn't if we wanted to perform a more disrupting event such as an engine shutdown.
This is much more serious than just the stage being slightly lighter than assumed.
With one (or more) engine(s) less, vehicle not only has worse performance, but also a completely different characteristic - for example, the current stage will take longer to complete its burn.
This significantly impacts UPFG as well as PEGAS internals:
it has already figured out when to ignite the subsequent stage, and with an engine down this prediction is simply wrong.

### How do we work around that?
This could all have been avoided, of course.
After all, both the jettison and engine shutdown events were known about ahead of time (at launch).
Why not just account for them prior to UPFG activation so that guidance can be calculated with taking all that into account?
This is exactly what PEGAS is doing now.
Let's see *how*.

If you look at [`pegas.ks`](../kOS/pegas.ks), after the passive guidance loop ends, a function `initializeVehicleForUPFG` is called.
This is defined in [`pegas_util.ks`](../kOS/pegas_util.ks), check it out as well.
The original purpose of this function was to calculate the initial mass of a sustainer-type stage.
Currently, it also handles some of the `sequence` events and incorporates their effects into the vehicle description.
That is, a pass through all of the events is being made, and if an "offending" event (currently only jettison and shutdown) is encountered,
its effects are calculated and `vehicle` config gets updated.

The way it is done is via introduction of "virtual stages".
The function starts by identifying which stage will be active during execution of the event.
Then it *splits* that stage into two:
* the original stage that fires **until** the event - this one remains the same, except that its burn time is reduced, and the remaining fuel is treated as dry mass now,
* a new stage is inserted that fires **after** the event - this inherits most of the properties of the original stage, with the exception of those that are altered by the event.

And so, after a jettison event, the dry mass (and consequently also the total mass) of the stage will be reduced by the mass lost in the event.
Similarly, after an engine shutdown event, the `engines` substructure will now contain less engines than the original stage, and the total burn time will be recalculated.

### What's the effect?
Introduction of these virtual stages does not influence the launch sequence in any way.
The only difference is that UPFG will automatically take them into account when computing the thrust integrals, and hence the entire guidance.

Does it eliminate the "jerk" when executing an event?
It may seem that it should, since now guidance "knows" that a change will occur at a specific point in time - right?  
Well, not exactly; you might still observe some sudden changes.
This is because UPFG, deep down, does not really treat "stages" as a discrete sequence, i.e. they are not individual and separate entities.
One of the first things it does is calculation of so-called "thrust integrals" - six numbers that attempt to describe the entire vehicle.
(A curious reader is encouraged to read my [in-depth explanation of the UPFG algorithm](https://github.com/Noiredd/PEGAS-MATLAB/blob/master/docs/upfg.md) - see sections on blocks 3 and 4 for information on thrust integrals.)
So no matter how many stages there are, one or five, the vehicle is considered in terms of these 6 values.
As you see, this approach is merely an approximation, and so it is not possible to make accurate adjustments for minuscule details such as "mass will be reduced by 1480 kg at T+242, and engine 5 will shut down at T+311".

Therefore you might still encounter slight "jerks" in pitch when executing an event (i.e. activating a virtual stage) or staging.
Beside that, you might (will) notice Tgo fluctuating somewhat.
Don't worry, it's perfectly normal.
A vehicle can start highly complex:
a "simple" 2-stage vehicle with one engine shutdown and one fairing jettison will expand to 4 stages after inclusion of the virtual ones.
For such a complex vehicle, the thrust integrals will be a rather gross approximation.
But as the flight progresses and the stages are jettisoned, the vehicle becomes simpler and the integrals become much better approximations of vehicle performance
(finally, for the last stage, they turn into a mathematically exact representation).
So, if your initial Tgo is calculated at T+655 seconds, then it drops to T+639, and then jumps to T+678 - don't be alarmed.
UPFG is simply doing its job and getting gradually better at it.

It's not perfect, but at least it allows us to shut down individual engines during the flight.
