# PEGAS
*Powered Explicit Guidance Ascent System*, from here referred to as *PEGAS*, is an ascent autopilot for Kerbal Space Program made and ran in [kOS](http://forum.kerbalspaceprogram.com/index.php?/topic/61827-122-kos-scriptable-autopilot-system-v103-20161207/), designed to control launch vehicles under a modified version of the game running [Realism Overhaul](http://forum.kerbalspaceprogram.com/index.php?/topic/155700-113-realism-overhaul).
Its unique feature is the implementation of a real-word rocket guidance algorithm: Unified Powered Flight Guidance, as used in the **Space Shuttle** GN&C computer for the standard ascent flight mode.
Short list of what PEGAS is capable of:
* estimation of a launch window,
* calculation of a launch azimuth,
* simple atmospheric ascent by pitching over and holding prograde with zero angle of attack,
* automatic guidance to orbits defined by:
  * apoapse
  * periapse
  * inclination
  * longitude of ascending node
  * or, alternatively, selecting an existing target,
* executing timed events (engine ignition, payload fairing jettison, or anything you want - via delegates),
* automatic staging, complete with ullage handling,
* ...and nearly anything else you want - via [addons](docs/addons.md)!

More info on my KSP [forum thread](http://forum.kerbalspaceprogram.com/index.php?/topic/142213-pegas-powered-explicit-guidance-ascent-system-devlog/), also see my [prototype repository](https://github.com/Noiredd/PEGAS-MATLAB).

## **[Version 1.2 is here!](https://github.com/Noiredd/PEGAS/releases/tag/v1.2)**

### How to use
1. Make sure you have [kOS](http://forum.kerbalspaceprogram.com/index.php?/topic/61827-122-kos-scriptable-autopilot-system-v103-20161207/) installed. Note: [basic](http://ksp-kos.github.io/KOS_DOC/language.html) knowledge of kOS will be very handy.
2. Dowload files from this repository's [kOS folder](kOS) and place them in your `Script` folder.
3. Define your vehicle and mission - see [tutorial](docs/tutorial.md) and [reference](docs/reference.md).
4. Once on the launch pad, load the definitions from pt. 2. and type `run pegas.` in kOS terminal.

### How to get help
PEGAS is not a magical do-it-all, it needs some effort to set up and get running.
It has been tested with several launch vehicles, from real-world launchers like Atlas V or Saturn V, through user-made vehicles, both in RO and vanilla settings.
However, I cannot guarantee that it will handle *any* vehicle or that it is entirely bug-free.
Likely, it will take you several tries before you get your rocket flying - and maybe you will find yourself unable to do that at all.
I am willing to provide support, correct bugs and (to some extent) introduce new functionalities to PEGAS.
In case of problems: read the [how to submit issues](docs/issues.md) page and then visit the issue tracker.

### Demo
Here is a video demonstration of the initial release of PEGAS in action, flying an Atlas V to a parking orbit aligned with the International Space Station.
It mostly focuses on explanation of the underlying guidance algorithm, only showcasing what functions PEGAS *has* instead of explaining how to use them.
For that I strongly recommend reading the [tutorial](docs/tutorial.md).

<a href="https://youtu.be/NEQD7AQoLXk" target="_blank"><img src="http://img.youtube.com/vi/NEQD7AQoLXk/0.jpg" width="240" height="180" border="10" /></a>
