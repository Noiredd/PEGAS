## PEGAS
*Powered Explicit Guidance Ascent System*, from here referred to as *PEGAS*, is an ascent autopilot for Kerbal Space Program made and ran in [kOS](http://forum.kerbalspaceprogram.com/index.php?/topic/61827-122-kos-scriptable-autopilot-system-v103-20161207/), designed to control launch vehicles under a modified version of the game running [Realism Overhaul](http://forum.kerbalspaceprogram.com/index.php?/topic/155700-113-realism-overhaul).
Its unique feature is an implementation of a real-word rocket guidance algorithm: Unified Powered Flight Guidance, as used in the **Space Shuttle** GN&C computer for the standard ascent flight mode.
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
* executing of timed events (engine ignition, payload fairing jettison etc.),
* automatic staging with ullage handling.

More info on my KSP [forum thread](http://forum.kerbalspaceprogram.com/index.php?/topic/142213-pegas-powered-explicit-guidance-ascent-system-devlog/), also see my [prototype repository](https://github.com/Noiredd/PEGAS-MATLAB).

### How to use - see [tutorial](docs/tutorial.md) and [reference](docs/reference.md)
1. Dowload files from this repository's [kOS folder](kOS) and place them in your `Script` folder.
2. Define your vehicle and mission.
3. Once on the launch pad, load the definitions from pt. 2. and type `run pegas.` in kOS terminal.

### Note about this repository
I have been using tabs throughout the whole code, having its length set to 4 spaces in all my editors.
I was unaware that github uses length of 8 - as a result, some of the `.ks` files look *really* bad.
If your eyes hurt, you can force github to display them with tab size of 4 spaces by adding `?ts=4` to the URL of the file you're viewing.
Unfortunately there is no way to make it a global setting (or even configure it for the repository).

### Disclaimer
This is a first public release of PEGAS.
Due to sheer amount of work on prototyping and coding it, and the range of potential problems with launch vehicles, I have been unable to test it with many rockets.
Therefore, I cannot guarantee that it will handle *any* vehicle or that it is entirely bug-free.
Likely, it will take you several tries before you get your rocket flying - and maybe you will find yourself unable to do that at all.
I am willing to provide support, correct bugs and (to some extent) introduce new functionalities to PEGAS.
In case of problems: read the [how to submit issues](docs/issues.md) page and then visit the issue tracker.

### Demo
<a href="https://youtu.be/NEQD7AQoLXk" target="_blank"><img src="http://img.youtube.com/vi/NEQD7AQoLXk/0.jpg" width="240" height="180" border="10" /></a>
