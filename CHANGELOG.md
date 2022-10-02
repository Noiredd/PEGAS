### Change log

## [v1.3](https://github.com/Noiredd/PEGAS/releases/tag/v1.3) "Olympus" (2022-10-02)
The final major version of PEGAS: now it has _all you might possibly use_.

##### Features:
* GUI overhaul: better reporting of the vehicle and guidance status,
including an event-by-event **flight plan** display (#37), which is possible because...
* UPFG is now aware of jettison and shutdown events *before* they happen (via a mechanism called [_virtual stages_](docs/magic.md))
* new passive guidance mode, `pitchProgram`, allows fine-tuning the ascent with a pitch-altitude schedule - **by [Aram1d](https://github.com/Aram1d)**
* new event type: `shutdown`, allows shutting down specific engines (by part tags), **also during active guidance**
* new event type: `action`, allows execution of standard action groups
* new ullage type: `hot`, allows hot-staging by reversing the jettison-ignition order
* new staging option: `postStageEvent`, allows jettison _after_ ignition (e.g. an interstage ring, Saturn-style)
* addons: you can now create custom extensions to PEGAS  
an [addon sharing repository](https://github.com/Noiredd/PEGAS-addons) has also been started

##### Tweaks:
* unscheduled loss of thrust will now be detected and handled by aborting the mission
* known errors in vehicle/controls configuration will now crash the system _prior to launch_
* improved error checking for vehicle staging configuration
* "jettison but no ignition" configuration on a first stage is now banned - instead, use either the `postStageEvent` option in `staging` or the jettison event in `sequence`
* settings are now grouped together in a lexicon instead of being a bunch of scatterred globals
* default settings have been changed to relax the overly strict convergence criterion and increase the initial convergence time

##### Fixes:
* PEGAS will now maintain min-AoA control while initially converging UPFG
* UFPG will not longer steal control the instant it converges, but actually wait until the moment defined by user in `controls["upfgActivation"]`
* throttle control in active guidance phase now correctly (i.e. instantly) switches between constant-acceleration and default modes
* countdown clock is now accurate
* documentation upgrades

##### Under the hood:
* unified events system (`pegas_events`): no more "user", "staging", and "system" events (#43)
* refactored the passive guidance loop, moving the atmospheric guidance part to a new module, `pegas_atmo`
* `initializeVehicle` renamed to `initializeVehicleForUPFG` and refactored
(`recalculateVehicleMass`, `accLimitViolationTime`, and `createAccelerationLimitedStage` have been merged into it)
* for legibility, `stagingInProgress` is now used consistently and several new technical flags are introduced (see [e8c051b](../../commit/e8c051b))
* version number is no longer stored in the UI code ;)


## [v1.2](https://github.com/Noiredd/PEGAS/releases/tag/v1.2) (2021-01-17)
New CSE routine, delegate events, and several minor fixes.

##### Features:
* new Conic State Extrapolation routine **by [pand5461](https://github.com/pand5461)**
* delegate events - call any function at any time via `sequence`
* cleaner settings system

##### Fixes:
* fixed a typo in the comms module
* cleaner pre-check routine with better feedback


## [v1.1](https://github.com/Noiredd/PEGAS/releases/tag/v1.1) (2017-11-05)
Multiple fixes for bugs identified after the initial release and new features requested since.

##### Features:
* inter-CPU communication system **by [Patrykz94](https://github.com/Patrykz94)**,
* roll control via user events,
* better throttle control via user events,
* new user event type (`jettison`) to account for the amount of jettisoned mass,
* stages can explicitly shut down engines (useful for Falcon 9 style missions),
* LAN and inclination as optional parameters,
* launch direction (northerly and southerly, as well as "nearest opportunity"),
* automatic seizing of control from the kOS CPU part.

##### Fixes:
* throttling obeys the way [Realism Overhaul works](https://github.com/Noiredd/PEGAS/issues/12),
* sustainer-type stages are now handled properly,
* minor code clean-ups, GUI and documentation fixes.


## [v1.0](https://github.com/Noiredd/PEGAS/releases/tag/v1.0) (2017-07-30)
Initial release of PEGAS in its current form (all old code has been removed), introducing the Unified Powered Flight Guidance (UPFG) algorithm.

##### Features:
* precise orbital guidance, allowing targeting both inclination and longitude of ascending node + periapsis and apoapsis,
* launches to reference plane (by selecting target in the in-game map),
* simple atmospheric ascent in 3 steps: vertical ascent for a given time, pitch-over by a given angle, hold prograde,
* automatic lift-off time and (optionally) launch azimuth calculation,
* flexible automatic staging allowing for separation-ullage burn-ignition sequences, with support for both SRB and RCS-based ullage (as well as none at all),
* basic user events system (for jettisoning of spent strap-on boosters, fairings etc.).
