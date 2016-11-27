## PEGAS
*Powered Explicit Guidance Ascent System* is an autopilot for Kerbal Space Program made and ran in kOS, designed to control launch vehicles under a modified version of the game running [Realism Overhaul](http://forum.kerbalspaceprogram.com/threads/99966). Its unique feature is an implementation of a real-word rocket guidance algorithm: previously [Powered Explicit Guidance](http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19660006073.pdf), more recently [Unified Powered Flight Guidance](https://ntrs.nasa.gov/search.jsp?R=19740004402) - as used in the **Space Shuttle** GN&C computer.

Right now PEGAS is undergoing a major rework: previously it implemented an old version of PEG, which (among its many limitations) only supported launching to circular orbits with no control over plane - constant azimuth was assumed throughout the mission. Now a full implementation of UPFG (in standard ascent mode) is being coded - this will allow it to target orbits not only of a given shape (apoapsis + periapsis), but also in a given plane (inclination + longitude of ascending node).

### MATLAB vs kOS
Since kOS is not a tool for algorithm design, PEGAS is being coded and tested in MATLAB. Hence, its code comes in two parts:
* MATLAB code - all of the most recent advances are to be found here,
* kOS code - right now contains only the old version of PEGAS (it should be working, but it's largely undocumented; so go ahead and give it a try but I will not provide any help on it now, as it will soon be replaced by an all-new implementation).
These parts are not interdependent - MATLAB is just a prototype, and obviously you need the software to run it, while kOS (is meant to be) a ready product you can plug into your game folder and send missions with.

### Current kOS version
In the kOS folder you will find an **obsolete version** of PEGAS that can only launch two-stage rockets into a circular parking orbit of a given altitude with no inclination control. It can be set to jettison boosters or fairings, and can perform a ullage burn using either RCS or SRBs. It is capable of igniting engines before launch, allowing liquid fuel engines to reach and stabilize thrust before liftoff. Its main limitation aside from targeting capabilities is: you need to supply a pitch program for the first stage guidance.

*Check how it performs in my youtube video below:*

<a href="https://youtu.be/0LGAizO-6K4" target="_blank"><img src="http://img.youtube.com/vi/0LGAizO-6K4/0.jpg" width="240" height="180" border="10" /></a>

### Recent advances

PEGAS now uses a very general guidance algorithm, the Unified Powered Flight Guidance. The MATLAB prototype is an almost complete implementation (with just one missing part replaced by a brute-force method - everything else works), with kOS code scheduled to be ported shortly. Brief list of most important features includes:
* targetting circular orbits with an altitude and plane constraint (inclination + longitude of ascending node), elliptic orbits coming soon
* multistage vehicle support with constant-thrust and acceleration-limited mode
* easier first stage guidance - pitch programming possible, but a simple "pitch over and hold prograde" also supported
* automatically estimated launch window basing on launch site position and target orbit
* no complicated pre-flight analysis needed - as long as the vehicle has enough power, missing the window slightly will not affect insertion precision
Please note that those are only features of MATLAB prototype that is still under final phases of development.

### Credits

MATLAB `linearFit.m` function contains pieces written originally by [Andrey Rubshtein](http://stackoverflow.com/users/817452/andrey-rubshtein) and [Nikolai Golovchenko](http://golovchenko.org). `flightPlots.m` uses [Richard Crozier](http://www.mathworks.com/matlabcentral/profile/authors/1590682-richard-crozier)'s [`tightfit`](http://www.mathworks.com/matlabcentral/fileexchange/34055-tightfig) (see MATLAB\tightfit.license for attached BSD license). I also made use of [Will Campbell](https://www.mathworks.com/matlabcentral/profile/authors/1050816-will-campbell)'s [`earth_sphere`](https://www.mathworks.com/matlabcentral/fileexchange/27123-earth-sized-sphere-with-topography) code (see MATLAB\earth_sphere.license for attached BSD license). Thank you for your great work!
