## PEGAS tutorial
Jump to:
* [Introduction](#introduction)
* [Basics](#basics)
* [Controls](#controls)
* [Vehicle](#vehicle)
* [Sequence](#sequence)
* [Mission](#mission)
* [Summary](#summary)
* [Example](#example)
* [Note for kOS beginners](#note-for-kos-beginners)

---

### Introduction
PEGAS is an ascent autopilot which aims to support a wide variety of vehicles in a wide variety of missions.
It uses the real-world Unified Powered Flight Guidance algorithm for precise and fuel-optimal orbital guidance.
You can read more about the algorithm [here](https://github.com/Noiredd/PEGAS-MATLAB/blob/master/docs/upfg.md), but for a crash course you only need to know the following:
* UPFG works best outside atmosphere (that is, it is oblivious to atmosphere at all),
* UPFG requires complete knowledge of the vehicle it is to guide (stage by stage),
* UPFG does not know the concept of *coasting* (the shorter the pauses between stages, the better),
* UPFG is not magical and can't do the impossible (direct equatorial orbit from KSC, launch to an orbit that your vehicle can't physically reach etc.).

The ascent with PEGAS is divided into 3 phases.
In the **pre-launch** phase, the launch time and azimuth are calculated, and the vehicle sits on the pad patiently waiting.
In the **atmospheric ascent** phase, the vehicle goes up for some time, then pitches over by some angle, and once the velocity vector matches its attitude, it keeps prograde (a.k.a. performs a simple gravity turn).
Finally, in the **closed-loop guidance** phase (or *active guidance*), the UPFG algorithm controls the vehicle, automatically guiding it onto a desired trajectory.  
All of those phases require information from *you*. You need to input the parameters of the gravity turn, information about your vehicle and its intended destination.
In this short tutorial, I will explain how to provide that information.
Reading is strongly encouraged, as PEGAS is quite complex.

PS: Basic knowledge of kOS syntax is recommended - if you find yourself struggling to understand what's going on, do [read those](http://ksp-kos.github.io/KOS_DOC/language.html).

### Basics
When you `run pegas.`, it is assumed that all the information and the vehicle is ready to go, and the mission is to begin.
Before you do that, you must create 4 structures (as global variables):
* `vehicle` - contains information about your vehicle, stage by stage,
* `controls` - defines parameters of the launch, the gravity turn and the moment of phase change,
* `sequence` - lists all the special events that will happen during the flight (such as payload fairing jettison),
* `mission` - your payload and desired orbit.

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
If you want something else to happen (eg. during the atmospheric ascent), you need to manually specify it using `sequence` (read on).

In the following 4 sections I will describe the use and meaning of each of the 4 variables you have to create.
This tutorial is a conceptual overview of what you need to do - for details of those variables, do check the headline for each section, as it links to the respective section in the reference.

### [Controls](reference.md#controls)

This variable controls behavior of the vehicle prior to and during the atmospheric ascent.  
I specifically mention it first because it is crucial to successful flight, and the most difficult one to get right.
As summarized in the reference, it holds 4 keys. Their meanings are as follows:

##### launchTimeAdvance
UPFG allows targeting specific orbital *planes*.
Obviously, launch timing is crucial in order to hit the right plane - intuitively, we know that the launch must occur near the moment that the launch site rotates under the target orbit.
We cannot launch directly when it happens though - the vehicle inherits the Earth's circumferential velocity and continues drifting east, past the orbit.
When UPFG kicks in, it tries to correct for that drift by aiming more towards west than would otherwise be necessary.
Sometimes it can recover from that, other times not - but the more *late* the launch was, the more performance degradation this causes; eventually, vehicle might not have enough fuel to make it to orbit.
Similar situation occurs if the launch was timed too early - UPFG points the vehicle more to the east, to make it *catch up* with the target plane, wasting fuel in the same manner.  
This is a difficult parameter to get right, but fortunately as long as you're in a reasonable range, it only hinders performance (instead of causing critical failures).
Experimentally, I found that times on the order of 120-150 seconds work well.
Vehicles that reach orbit faster will need less advance time, while those long burning upper stages might use more.

##### verticalAscentTime and pitchOverAngle
Atmospheric ascent is difficult from a guidance standpoint, and doing it automatically is far beyond the scope of this project.
Instead, you have control over it using two parameters.
The process is simple: after a certain *time* after liftoff (`verticalAscentTime`), your vehicle will pitch over by a certain *angle* (`pitchOverAngle`).
It will hold this attitude until the velocity vector matches it, and then it will lock to prograde, so as to minimize the angle of attack (and hence risk of a RUD).

Rules are that the earlier you pitch over and by the larger angle, the shallower your ascent.
In the worst case, your vehicle will fall back to the ocean (or, more likely, get torn apart due to flying too fast too low).
On the other hand, if you pitch over too little and too late, your gravity turn will not *turn* you enough, and your vehicle will be going too vertically.
Additionally, the more TWR your vehicle has, the sooner and more aggressively you can turn - a good overview on that is given by [ferram in the RO wiki](https://github.com/KSP-RO/RealismOverhaul/wiki/Ferram-on-Ascent-Profile-and-TWR).

This is a doubly important parameter, since it directly influences the active guidance phase.
It is difficult to give any clues regarding what your altitude and apoapse should be when the atmospheric ascent ends, because different vehicles behave differently.
Generally though, if you target a 200 km orbit and UPFG engages when you're on 45 km with a 50 km apoapsis, or on 120 km with 350 km apoapsis - it's a hint that your gravity turn was too shallow/too steep.
In extreme cases, UPFG will refuse to converge and your mission will fail.

**How to get these right?**
First, you need to figure out some rough values by trial and error.
If you get UPFG to converge and it doesn't need to pitch 60 degrees from the current prograde, you're in the ballpark.
When UPFG converges, look at the pitch it calculated - to simplify things: the closer the active guidance pitch was close to the current prograde, the better your atmospheric ascent was.
If it pitched significantly above prograde, it means that your ascent was too shallow and UPFG needs to gain some more vertical velocity - reduce the `pitchOverAngle` or increase the `verticalAscentTime` slightly.
If it pitched way below (it may even point below the horizon in extreme cases!), your ascent was probably too steep - adjust parameters in the opposite manner.

Getting them right might take several attempts.
Getting them perfect (so that the UPFG-generated pitch matches prograde) might turn out impossible, and is not really worth trying.
Delta-v losses from sub-optimal steering are not something you need to worry about unless you're trying to fly missions on the absolute limits of capability of your launch vehicle.  
Tuning might prove particularly difficult if you're getting low ingame FPS during liftoff, or your launch clamps misbehave.
Unpredictable separation that disrupts your vehicle from flying straight can even make good settings randomly fail - beware.

##### pitchProgram
For some vessels the above solution (with `verticalAscentTime` and `pitchOverAngle`) does not give enough controllability.
PEGAS provides another approach to guide your vehicles through the atmospheric climb with more accuracy: `pitchProgram`
It allows you to define a precise pitch-vs-altitude program by specifying two lists (as keys of the `pitchProgram` lexicon):
* `altitude` is a list of "keypoint altitudes",
* `pitch` is a list of desired pitch angles,
i.e. at any altitude from the `altitude` list,
pitch angle will be selected from the corresponding element of this list.

PEGAS will perform linear interpolation between the given pairs of values,
calculating pitch as a function of altitude.
Each altitude/pitchAngle pair will result in one linear segment of the ascent trajectory.

**Note**: the first `altitude` entry is of special importance.
Prior to reaching this altitude, the vehicle will fly at 90 degrees pitch (perfectly vertically) -
_no matter what the corresponding pitch is_ (although it is recommended to set it to 90).
Only after this altitude has been reached,
the linear interpolation kicks in and the vehicle starts following your pitch program.

Both solutions require that you know your rocket to define at least one set of values.
For more information see the [pitchProgram](reference.md#pitchprogram) section of the reference.
For an example usage see [SoyuzTMA.ks](../kOS/boot/SoyuzTMA.ks) sample.

##### upfgActivation
This is when the atmospheric ascent ends, and active guidance begins.
You want to be outside the atmosphere when that happens, 40-50 km is good.
From this moment the UPFG is in control over your vehicle, and if you're too low and it decides to pitch up too much, you might experience your vehicle tumbling out of control for a moment, or even rapidly disassembling.
It's difficult to provide example numbers, as this variable strongly depends on your vehicle - see the [boot files](../kOS/boot).

##### Rolling
Roll control in PEGAS is achieved using two mechanisms.
First roll maneuver occurs together with the pitchover, by angle given via `initialRoll` key.
By default (in case the key was not present) vehicle will roll to 0 degrees angle.
In any time during the flight, roll can be changed during a preprogrammed event (see [`sequence`](#sequence) below).

### [Vehicle](reference.md#vehicle)
Physical description of each stage of your vehicle, *as seen by the UPFG algorithm*.
That means, **only those stages that will be actively guided need to be listed here**.
For each of them, you are supposed to provide basic physical information (masses, [engine](reference.md#engines) parameters), as well as details of the [*staging*](reference.md#staging) procedure.
The basic info should contain (see reference for details):
* total mass of the **entire vehicle** at the moment of ignition of each stage,
* mass of the propellant in each stage,
* specific impulse and thrust (both in vacuum) of each vehicle in each stage.

This means that stages are not exactly independent entities, but rather *states of the vehicle*.
Mass of each stage must include masses of all successive ones.
You **do not** have to include mass of the payload in it - PEGAS automatically does that, if you define it in your `mission` variable (see below).

"Staging" is to be understood as hitting spacebar in a timed order.
It happens when a currently flying stage burns out - but **beware**!
PEGAS does not dynamically check *anything* - the only way it knows that the stage has burned out, is because it divided the mass of the fuel it had by the total flow of its engines.
If you input wrong numbers, e.g. your vehicle has more fuel than the system thinks it has,
PEGAS will gladly attempt to execute a staging procedure with the previous stage's engines still burning
(if you're bold, you can use this to your advantage).

The staging logic is as follows: when a stage is activated, its staging sequence is executed, and the following things happen:
* if the previous stage has to be jettisoned as a separate event, some time is waited and spacebar is hit,
* if the current stage needs engines to be explicitly ignited, some time is waited and ullage sequence starts:
  * if no ullage burn is needed, hit spacebar directly (to ignite the engines),
  * otherwise, ignite the ullage motor depending on the kind of burn:
    * if ullage uses solid rocket motors, hit spacebar,
    * if ullage uses RCS, engage it and push forward;
  * wait some time while the ullage push is in progress,
  * hit spacebar (to ignite the engines),
  * if the ullage is done with RCS thrusters, keep pushing for some time, then disengage them,
* if some extra action was requested, more time is waited and spacebar is hit again.

This provides a way to give your stage some "buffer time" before separation - eg. if you're not sure if you got the masses right, you can add some more time before ignition of the next stage.

##### Important note about the first stage
Let's say you're flying a Saturn V.
A natural choice would be to align the moment of UPFG activation with ignition of the second stage.
This way, you can use the [`staging`](reference.md#staging) facility in its description to make PEGAS control the ignition and separation of the previous stage.
In this case, you write down the total mass of the stack after separation of the first stage, and the mass of fuel in the second one.
You set the `upfgActivation` key in `controls` to time of burn of the first stage.

By contrast, you could be flying an Atlas V, and waiting till its booster burns out is not an option.
You can define `upfgActivation` so that UPFG is engaged while the booster burns, and specify no ignition event for that stage.
It *would be* quite cumbersome to have to calculate the vehicle's mass at that point before the flight - but PEGAS has one trick to make it easier.
If a first actively guided stage requires no ignition, it assumes the stage has been burning from time zero: it checks the current mass, compares that with the dry mass of the stage and infers the amount of fuel left in the tanks.
In this case, the only thing you need to do is provide that dry mass, and set the `upfgActivation` to virtually any moment.

##### Jettison but no ignition?
Following the above Atlas V example, you might think that it would save you some work on the configuration, if you dropped the SRBs and activated UPFG at the exact same time,
(simply by stating `"jettison", TRUE`, but `"ignition", FALSE` in the [`staging`](reference.md#staging) entry).
This will **FAIL** your mission!
PEGAS will measure the mass of the vehicle **with** the SRBs, while your vehicle definition must be stated without including them anymore.
Therefore, PEGAS will assume the SRB mass is *fuel* and erroneously calculate the burn time of the sustainer.

You should AVOID stages with `"jettison", TRUE, "ignition", FALSE` in general.
If you only need to jettison something, use the `sequence` (see below).
Similarly, you should AVOID making any changes to your vehicle (e.g. staging something away)
shortly before engaging UPFG - especially if your vehicle is based on an Atlas V-like sustainer.

##### Note about constant-acceleration phases
PEGAS has the capability to run vehicles with acceleration-limited stages (eg. Space Shuttle, Atlas V).
You do that by specifying `gLim` in your stage description (in multiples of `g0`, standard Earth gravitational acceleration).
In order for that setting to work properly, you have to also specify `minThrottle` to inform PEGAS about the throttling limit imposed by the stage's engines.
It is required due to how Realism Overhaul calculates throttle - the setting is not absolute (ie. 50% slider *does not* in general equal 50% throttle), but relative to the throttle range of the engine.
That is, if an engine throttles in 40-100% range, setting the slider to 50% will actually result in (100%-40%)\*0.5 + 40% = 70% throttle.
Therefore, PEGAS **must** know what is the throttle limit in order to calculate throttle commands correctly.

Additionally, you have to pay attention to your vehicle's mass.
PEGAS makes a burn time prediction based on what is the vehicle's mass, and schedules the next stage separation/ignition basing on that.
If the actual vehicle's mass turns out to be lower than the value in the `vehicle` structure, the resulting throttle commands (which are always based on the current *measured* mass) will be smaller than predicted.
This will cause lower than predicted fuel consumption, and consequently longer than predicted burn time - in the worst case, the next stage will separate while the current one is still burning, causing the two to collide.  
Even something as innocent as jettisoning the payload fairings can have grave consequences if you haven't prepared your vehicle for that.
PEGAS provides you with a special tool: the `jettison` event, which allows you to inform the system of the mass lost during the event (see below).

### [Sequence](reference.md#sequence)
This is how you control timed events, like:
* separation of the strap-on boosters,
* jettisoning the payload fairing,
* rolling to given attitude,
* throttle (only in the atmospheric ascent phase),
* shutdown of a specific engine (by tag),
* toggling action groups,
* execution of custom functions (kOS [delegates](http://ksp-kos.github.io/KOS_DOC/language/delegates.html)).

See the [reference](reference.md#sequence) for a list all possible events and how to use them.

As you see, both `sequence` and `vehicle` can cause a staging (equivalent to hitting spacebar).
The main difference between `vehicle` staging and `sequence`, is that vehicle staging events are bound to the *physical parameters of the vehicle* (how much fuel does a stage have, how fast does it consume it => when does the next stage activate), while `sequence` events are bound directly to time (counted from lift-off).
For this reason, you must pay attention that your timed events be properly aligned in the in-game staging sequence, with respect to the staging events.
You don't want your payload fairing jettison event to accidentally separate the currently burning stage.
Recommended approach is to avoid scheduling other staging events near the main vehicle staging.

One unique thing about sequence is that it controls the lift-off too.
You **need to** have an entry at time zero that releases the launch clamps.

##### Note about delegate events
If you like to organize your code into boot files with vehicle configs and mission scripts with target parameters, you will run into a problem that's well explained in [this section](https://ksp-kos.github.io/KOS_DOC/structures/misc/kosdelegate.html#attribute:KOSDELEGATE:ISDEAD) of kOS documentation.
In short, a delegate only lives as long as the script that it comes from.
After you've dropped out of that script, the delegate is "dead" (`KOSDelegate:ISDEAD`) and any attempt to call it will **error out** the kOS interpreter.
This means you should define all delegates you want to execute via `sequence` in the same script that starts PEGAS (`RUN pegas.`).

### [Mission](reference.md#mission)
All previous variables can be understood as specific to the launch vehicle; this one is specific to each mission.
It defines the target orbit (or, target *state* - nothing stops you from performing an ICBM-like launch to a suborbital trajectory), and the mass of payload to deliver there.
PEGAS supports launches to orbits defined by a reference body (selecting target in the universe map).
However, this way you can only define the target **orbital plane** - apoapse and periapse still need to be entered by variable.

Important note: selecting target will only work for bodies orbiting the Earth (or any body that you're launching from\*).

\* - none tested except Kerbin and Earth.

### Summary
1. Set up your `vehicle` variable, defining each stage starting from the one that will be burning when the UPFG kicks in.
2. Set up `controls`, using [ferram's guide](https://github.com/KSP-RO/RealismOverhaul/wiki/Ferram-on-Ascent-Profile-and-TWR) and experiment.
3. With the atmospheric ascent phase set, and active guidance staging events as well, check if you have any events that are not covered in either.  
Need to drop SRBs while in atmosphere?  
Jettison paylod fairing when you're confident the altitude is high enough?  
Set up the `sequence` accordingly.  
Bear in mind that it needs to have at least one element: the release clamps command at time zero!  
Those steps get your vehicle ready (you may enclose them in a single script and bind it as a boot file to your vehicle).
4. Now the only thing you need to do is specify where you want it to go: define your `mission`.
5. When in kOS terminal, load those 4 variables (boot files or by simply running scripts).
6. `RUN pegas.`

If something is going wrong mid-flight, you can use the standard action group `ABORT` to make PEGAS relinquish all control and exit.

##### Suspected bug in kOS
Note that if you revert flight while controls are locked by kOS, the attempt to lock them again (in the new flight) will result in `object reference not set` error.
When that happens you will have to leave and reenter the vehicle view.
For experiments involving PEGAS, it's best to `ABORT` flight using the standard action group (hit backspace) before you revert.

---

### Example
#### Simple sustainer rocket
Suppose we're building a medium launch vehicle, with a kerosene-oxygen first stage powered by 3 NK-33's and a hydrolox upper stage with 3 RD-0146's.
Furthermore, the second stage has a few small SRM's for ullage attached to it.
The rocket stands 343 800 kg heavy in total, which breaks into:
* first stage: 290 773 kg (198 163 kg liquid oxygen, 75 921 kg kerosene, 16 689 kg dry mass)
* second stage: 36 833 kg (27 440 kg liquid oxygen, 4 651 kg liquid hydrogen, 100 kg HTPB, 4 642 kg dry mass)
* payload fairing: 2 694 kg
* payload for testing: 13 500 kg

Our staging sequence in the VAB looks roughly like this:
* ignition of the NK-33's
* launch clamps release
* booster decoupler
* ignition of the ullage SRMs
* ignition of the RD-0146's
* payload fairing jettison
* payload detachment

Kerbal Engineer Redux tells us that those NK-33's will burn through the kerolox in 190 seconds, and the stage has approximately 4.4 km/s $\Delta v$.
This means that it will burn out high above the atmosphere, so we'll probably want to transition into active guidance earlier than that -
halfway through the first stage burn is a good guess.

Let's start by defining the `vehicle` -
remember, here we're thinking about what the UPFG sees, not necessarily what the vehicle looks like in the VAB.
Since we want UPFG to activate *during* the first stage burn, we need to define two stages.
First will be this heavy kerolox booster, *and everything on top of it*.
So the `lexicon` describing this stage should have the following keys:
* `massTotal` = 290773 + 36833 + 2694 (note how we don't add payload mass here)
* `massFuel` = 198163 + 75921
* `engines` = `LIST(LEXICON("isp", 331.0, "thrust", 3*1766000))` (vacuum Isp and thrust of an NK-33 multiplied by 3)
* `staging` = `LEXICON("jettison", FALSE, "ignition", FALSE)` (UPFG will be activated while this stage is already burning, so no need to do anything)

Now, for the second stage:
* `massTotal` = 4642 + 2694 (assuming we're still carrying the fairings)
* `massFuel` = 27440 + 4651
* `engines` = `LIST(LEXICON("isp", 463.0, "thrust", 3*98100))` (vacuum performance of 3x RD-0146)
* `staging` is a lexicon again, but this time it's going to be a little more complicated, as we need to do a few things here; let's list the keys one by one:
  * `jettison` = `TRUE` (because we do have to start with dropping the first stage)
  * `waitBeforeJettison` = 3 (just a little pause for safety, making sure the stage really burns out before we start)
  * `ignition` = `TRUE` (because we do have some engines that need to be ignited as a separate staging action)
  * `waitBeforeIgnition` = 2 (a slight delay to let the booster separate neatly - matter of taste)
  * `ullage` = `"srb"` (because we have solid rockets for ullage)
  * `ullageBurnDuration` = 2 (let's say this time will suffice to settle the fuel so we can ignite the main engines safely)

Pack those two lexicons into a list named `vehicle` and this part is done. Phew!

That's not everything though.
We still haven't accounted for things like ignition of the NK-33's on the pad,
releasing the launch clamps or jettisoning the payload.
For those we'll need `sequence`.
Sequence is a list containing "events" - lexicons describing what, when and how.
Remember, we **always** have to have an event at T=0 to execute the liftoff.
But in RO we want to ignite our engines before that.
So our sequence might look like this:
* `LEXICON("time", -4, "type", "stage", "message", "NK-33 ignition")` (ignition *before* liftoff)
* `LEXICON("time", 0, "type", "stage", "message", "LIFTOFF!")` (mandatory entry)
* `LEXICON("time", 200, "type", "jettison", "massLost", 2694, "message", "Payload fairing jettison")`  
(finally we drop the fairings; note how we're using type `jettison` and not just `stage`, even though they do the same thing in-game (i.e. hit spacebar to stage) - this `massLost` key will update the second stage definition by subtracting the jettisoned mass)

We're almost done, the only thing missing is the `controls`.
This is the most arbitrary input however as you'll need to tune it for each vehicle,
so we'll go through it only briefly.
* `upfgActivation` = 100 (as mentioned above, we'll want to activate roughly halfway through the burn -
too late is bad, too early is even worse)
* `launchTimeAdvance` = 120 (experiment with that if you need to)
* `verticalAscentTime` = 11
* `pitchOverAngle` = 2.5 (those two values you need to find for each vehicle yourself)
* `initialRoll` = 90 (matter of taste, e.g. sometimes you need to correct for misaligned control part)

And that's it for the vehicle part.
In order to fly it, all we need to do is set the `mission` structure,
load everything into terminal and hit `RUN pegas.`.
Among the things we could put into that structure would be `"payload", 13500`
to account for the last bit of mass that we haven't yet put anywhere.

#### Solid rocket boosters
Suppose we want to improve the above vehicle by adding some SRBs to it.
We pick the GEM-40, which weighs 12 962 kg fully loaded and 1 196 kg burnt out,
and attach two of them to the booster, increasing the total mass of the vehicle to 369 724 kg.
Now where do the staging actions go?
Obviously, we'll want to ignite at the same time we lift off,
so the ignition event will go to the same action as the launch clamps release.
We also need to jettison them at some point, which is going to be a separate event.

How does our vehicle definition change due to that?  
Of course, we need to schedule the jettison, so we need an extra entry in the `sequence`:
* `LEXICON("time", 67, "type", "stage", "message", "Dropping the SRBs")`

What about the `vehicle`?
Since the change happens *before* activation of the active phase, **we don't need to do anything**!
At T+100, when the UPFG kicks in, the booster looks exactly like it did in the previous version.
For the same reason it doesn't matter if we use `stage` or `jettison` type event for this job.

#### 3-stage vehicle
Let's look at a different situation, in which we have a three-stage vehicle, maybe something like Saturn V.
We'll skip the technical details here, let's just assume that the first stage burns out after 120 seconds,
second stage burns for the next 400 seconds and gets the vehicle almost into orbit,
and the final stage performs the insertion.

In this case defining the first stage could feel like an unnecessary chore,
since if we chose to activate it at T+100 like in the previous example,
it would only be actively guided for about 20 seconds.
Therefore it might be simpler to align activation of the UPFG with the second stage.
The `vehicle` would then only contain entries for the second and third stages.
However, you need to remember about `staging` on the second stage!
`jettison = FALSE` and `ignition = FALSE` only worked in the previous example because the stage was already lit when UPFG kicked in,
which will not be the case here.

Rest of the `vehicle` and `sequence` will be built like before.
The only extra thing that needs to be remembered is to set the `upfgActivationTime` at the correct value
(just after the booster burns out).

---

### Note for kOS beginners
First, let's assume your KSP install folder is `KSP\` (this is where `GameData`, `Resources` and other folders reside).
kOS folder is in `KSP\Ships\Script\` - that is where you should put all PEGAS files.
Optionally, you can create a subfolder `boot` in there, if you want to use the [boot files](http://ksp-kos.github.io/KOS_DOC/general/volumes.html#special-handling-of-files-in-the-boot-directory) functionality of kOS, which I recommend.

In order to follow with this tutorial, I recommend starting with a new, empty script in `KSP\Ships\Script\` - let's call it `myVehicle.ks`.
In this script, you go with points 1-3 from the [summary](#summary) - a good reference for that is any of the example [boot files](../kOS/boot).
If you did everything right so far, `myVehicle.ks` should contain **three** global variables: `vehicle`, `controls`, and `sequence`.  
Now create a new empty script, let's say `myMission.ks`.
In there, perform the 4th step: declare your `mission`; here are some [examples](../examples/) how to do that.

Now you are ready to fly.
Recheck your staging sequence and put your vehicle on the launchpad.
Open the kOS terminal and type `SWITCH TO 0.` in order to be able to load the files you've just prepared.  
`RUN myVehicle.` loads your vehicle definition - alternatively, you could use that script as a bootfile, to get to that point automatically.  
`RUN myMission.` loads your mission definition.  
As the last thing, `RUN pegas.` - if you did everything right, after a while of loading you should see the PEGAS interface with a countdown.
If you forgot to specify some of the variables, PEGAS will notice that and crash on purpose (leaving an error message that should tell you what was missing).
If however you made some mistakes... there are very few safety checks, and PEGAS assumes that everything you did, you did on purpose.
