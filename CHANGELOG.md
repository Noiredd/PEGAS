### Change log

## [v1.2](https://github.com/Noiredd/PEGAS/releases/tag/v1.2) (2021-01-17)
New CSE routine, delegate events, and several minor fixes.

##### Features:
* new Conic State Extrapolation routine by [pand5461](https://github.com/pand5461),
* delegate events - call any function at any time via `sequence`
* cleaner settings system

##### Fixes:
* fixed a typo in the comms module
* cleaner pre-check routine with better feedback

## [v1.1](https://github.com/Noiredd/PEGAS/releases/tag/v1.1) (2017-11-05)
Multiple fixes for bugs identified after the initial release and new features requested since.

##### Features:
* inter-CPU communication system by [Patrykz94](https://github.com/Patrykz94),
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
