## PEGAS
*Powered Explicit Guidance Ascent System* is an autopilot for Kerbal Space Program made and ran in kOS, designed to control launch vehicles under a modified version of the game running [Realism Overhaul](http://forum.kerbalspaceprogram.com/threads/99966). At its current state it features two main sections:
* passive pitch control for atmospheric flight (first stage guidance),
* active pitch control using Powered Explicit Guidance algorithm for parking orbit targeting (second stage guidance).

They are connected by a common launch sequence, so PEGAS controls the complete launch vehicle ascent from countdown to orbital insertion.

### Code structure
PEGAS is comprised of 5 main elements: 
* *bootfile*, designed as individual for each launch vehicle, containing LV-specific stuff,
* *loader* that initializes the system (`pegas_loader`),
* *libraries* that contain often-used functions (`pegas_nav`, `pegas_lib`),
* *settings* which contain some technical variables like timeouts (`pegas_set`),
* and the main launch sequence control program, `pegas`, that carries out the whole procedure.

### Current capabilities
**For now** PEGAS can only launch two-stage rockets into a circular parking orbit of a given altitude, although with no inclination control. It can be set to jettison boosters or fairings, and can perform a ullage burn using either RCS or SRBs. It is capable of igniting engines before launch, allowing liquid fuel engines to reach and stabilize thrust before liftoff.

*Check how it performs in my youtube video below:*

<a href="https://youtu.be/0LGAizO-6K4" target="_blank"><img src="http://img.youtube.com/vi/0LGAizO-6K4/0.jpg" width="240" height="180" border="10" /></a>

#### First stage guidance

First stage control is very simple: the autopilot simply follows a precalculated pitch program, at each step interpolating from a pitch-time table it is given in the bootfile. The downside is that you need to prepare this table before the flight, effectively having to solve the ascent path problem. The actual shape of the path will depend on the LV's parameters such as TWR and drag curve, as well as miscellaneous design constraints (apoapsis at burnout, drag losses, maxQ etc.) and will need to assure aerodynamic safety of the vehicle (too much AoA will kill it).

Please note that this project **does not** deal with ascent path design.

#### Staging sequence

Almost everything in PEGAS is timed, that is: no dynamic checking for engine shutdown, flow stability etc. are performed and most actions must be scheduled beforehand. Staging sequence is the best example of this. All events PEGAS can handle - separation, ullage engine ignition, main engine ignition, ullage engine cutoff (optional for RCS ullage) - are explicitly stated in vehicle-specific time table. All the autopilot does is execute them, assuming that they are all properly ordered in staging sequence (in other words: presses space when it's told to).

#### Second stage guidance (PEG)

Some time after seconds stage ignition (another declared value) PEG guidance is activated and the autopilot begins navigating the craft into a specified orbit. For now it can only be a circular orbit with only inclination coming from launch site latitude.

To understand the guidance algorithm, read the original document: [Explicit guidance equations for multistage boost trajectories](http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19660006073.pdf) by Fred Teren, Lewis Research Center. This article on [Orbiter Wiki](http://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance), which is less mathematical and focuses on what you need to know to implement it, will be easier to digest. However, it might not suffice to understand everything, and sometimes you'll need to refer to the original anyway.

### Coming up

* Currently PEGAS suffers some minor targeting accuracy problems (+/- 10km on a 200km target orbit). This is unexpected, since a prototype designed in MATLAB showed excellent performance, and kOS version is an exact translation of the code.

* Very important thing to have next is yaw control and inclined orbit targeting. I will be focusing on this feature now, first building a separate prototype in MATLAB.

* In the future, I will try to implement multistage control for PEG flight. This will allow for flying 3 stage vehicles like the Saturn V, or "2.5" stage Atlas-Centaur-style sustainer+second stage combinations.
