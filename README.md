## PEGAS
*Powered Explicit Guidance Ascent System* is an autopilot for Kerbal Space Program made and ran in [kOS](http://forum.kerbalspaceprogram.com/index.php?/topic/61827-122-kos-scriptable-autopilot-system-v103-20161207/), designed to control launch vehicles under a modified version of the game running [Realism Overhaul](http://forum.kerbalspaceprogram.com/threads/99966). Its unique feature is an implementation of a real-word rocket guidance algorithm: previously [Powered Explicit Guidance](http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19660006073.pdf), more recently [Unified Powered Flight Guidance](https://ntrs.nasa.gov/search.jsp?R=19740004402) - as used in the **Space Shuttle** GN&C computer.

Right now PEGAS is undergoing a major rework: previously it implemented an old version of PEG, which (among its many limitations) only supported launching to circular orbits with no control over plane - constant azimuth was assumed throughout the mission. Now a full implementation of UPFG (in standard ascent mode) is being coded - this will allow it to target orbits not only of a given shape (apoapsis + periapsis), but also in a given plane (inclination + longitude of ascending node).

### Recent advances
**IMPORTANT**: PEGAS repository has been split into two: [PEGAS-MATLAB](https://github.com/Noiredd/PEGAS-MATLAB) contains the prototype written in MATLAB, with all its bells, whistles and sophistication. This repository ([PEGAS](https://github.com/Noiredd/PEGAS)) *will contain* kOS code, ready-to-use for your launches.

PEGAS now uses a very general guidance algorithm, the Unified Powered Flight Guidance. Brief list of most important features includes:
* targetting orbits with periapsis/apoapsis and **plane** constraints (inclination + longitude of ascending node),
* support for multistage vehicles with constant-thrust and acceleration-limited modes,
* simple atmospheric stage guidance in a "pitch over and hold prograde" mode,
* automatically estimated launch window basing on launch site position and target orbit,
* no complicated pre-flight analysis needed - as long as the vehicle has enough power, missing the window slightly will not affect insertion precision.

### Preview
You can see the old version of PEGAS in my youtube video below. This will be soon replaced by a new thing, once it's ready.

<a href="https://youtu.be/0LGAizO-6K4" target="_blank"><img src="http://img.youtube.com/vi/0LGAizO-6K4/0.jpg" width="240" height="180" border="10" /></a>
