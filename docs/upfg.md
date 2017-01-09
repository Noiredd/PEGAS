## Unified Powered Flight Guidance

### Introduction
There are 3 essential problems with launching a rocket into orbit: determining when should it lift off, how should it travel through the atmosphere, and finally: how to make sure it reaches a precisely defined target orbit.
In this article we will focus on the last part, that is exoatmospheric guidance of a rocket vehicle, explaining one of the most general and well documented algorithms: Unified Powered Flight Guidance.
It was designed specially for the Space Shuttle orbiter, allowing it to carry out tightly constrained missions (most notably the Hubble Space Telescope service missions or ISS construction - both requiring precise rendesvous targetting) while providing a wide range of abort capabilities.

The following assumptions will be made (following the implicit assumptions of the original paper): vehicle is already in motion, high enough that aerodynamic drag can be neglected; only gravity and thrust affect it; there are no coast periods between stages.
The task is to steer the vehicle from its initial state (position, velocity) into a target state (orbit with given inclination, longitude of ascending node, semi-major axis, eccentricity and potentially also argument of periapsis) using only its engines.
In other words, we need to find pitch and yaw angles throughout the whole ascent, as well as determine the moment of engine cutoff.
Additionally, we want to do that in an *optimal* way - making sure as little fuel as possible gets wasted in any of the stages.

### Difficulties
For start, let's look into the forces that act upon the vehicle.
Thrust, although (let's assume for now) constant, causes a non-linear acceleration, because in order to generate it the vehicle must lose mass.
The math is easy thanks to [Konstantin Tsiolkovsky](https://en.wikipedia.org/wiki/Tsiolkovsky_rocket_equation) who developed solutions over a hundred years ago.
Then there is gravity, which is also pretty easy (at least in the 2-body problem) as all the math has been known for 3 centuries.
The problem arises when we try to integrate the trajectory (that is, predict the future state) of a body in the presence of **both** these forces.
If we rewrite the equations so that accelerations replace forces, we get thrust as a function of time (inverse linear) and gravity as a function of position (inverse square).
What we want is closed-form solutions: velocity (first integral of acceleration) and position (second integral) as functions of time.
Unfortunately, the equation we've just written cannot be generally solved this way, which means we cannot simply calculate the final state of a thrusting vehicle using a simple formula.
Difficulty number one: how do predict where the vehicle will be in the future, given some thrust vector and gravity?

Second major problem comes from the way we define our initial and target states.
At the beginning the vehicle's state is simplest described using 2 vectors in Cartesian (XYZ) coordinates: position and velocity.
But our target state is an orbit, not a single XYZ point - we would like to use [Keplerian elements](https://en.wikipedia.org/wiki/Orbital_elements) for that.
After all we do not really care *where exactly* will the vehicle be, as long as it's on the right ellipse in the right plane.
And if there is no particular point in space, there's also no particular velocity vector we need to achieve (it changes depending on position on the ellipse).
Difficulty number two: how to formulate the math if we don't have a common coordinate frames for input and output?

By the way, how will our answer even look like?
Let's forget the integration problem for a moment (we can always employ some numeric method to find a solution, or compute it with brute force) and assume we *can* predict the future state of a vehicle.
We know how the gravity will change (always points towards the center of a coordinate frame), but how will our thrust vector change?
Obviously, we are not interested only in pitch & yaw angles *right here, right now* - we have to somehow express them in a way that allows them to change, and in a way that allows us to integrate this change.
Most logical assumption would be to parametrize them as functions of time (as we are looking for all other solutions as independent functions of time), but then how do we choose those functions?
This is difficulty number three.

Finally, even if we formulated the whole math apparatus to let us predict the future state of a thrusting vehicle in a gravity field, we would only have a set of equations that inputs current vehicle state, gravity parameters and thrust function (pitch & yaw and magnitude) and outputs a new state in some time from "now".
We need something that works the other way around: inputs current and desired states and outputs pitch & yaw functions.
Final difficulty: how do we reverse those equations, so they yield what we need?

Additionally, it's worth mentioning that we cannot use a half naive, half brute-force approach of precalculating a trajectory on an external (potentially very powerful) computer and then just have a vehicle follow it.
Even the slightest deviations from that ideal path will accumulate on our way, along with errors from the unpredictable elements of the system: imprecise thrust vector control, thrust magnitude variations, turbulent air flow, imperfect gravity model - all can only be approximated, not predicted exactly.
Thus, our guidance system must generate commands on-the-fly, basing on measured vehicle state.
That imposes another difficulty, if we consider just how little computing power is there on a launch vehicle: the system must be simple and work as fast as possible.

### Solution
##### Coordinate frame
We will tackle the second difficulty by deciding on a non-rotating Earth-centric XYZ coordinate frame: X and Y axes will lie in the equatorial plane (X aligned with the 0° meridian at some given time), Z will point north.
It's easy to convert spherical coordinates (longitude, latitude & altitude) into XYZ - we just have to remember that the Earth constantly rotates, thus the launch site coordinates will also be continuously changing.
Converting Keplerian coordinates of the target orbit into XYZ is more difficult.
Since it is not a single, unique point in position&velocity space but rather a set of points, we can only express the desired state in terms of constraints.
We will write them in the following form:
* plane - defined by desired inclination and longitude of ascending node,
* altitude - desired radius at MECO,
* velocity - magnitude of the desired velocity vector,
* flight path angle - angle between desired velocity vector and local horizontal.

The first constraint can be easily and uniquely written in XYZ form as a vector normal to the target plane (we can use Euler formulas to calculate it knowing INC and LAN angles), explicitly describing where the orbit will be and what will be the direction of motion.
Next three describe what the orbit will look like: knowing altitude (radius) and corresponding velocity and angle, we can draw an ellipse in a unique way.
There's just one thing we cannot define: how will the orbit be rotated within the target plane (or, if we set the flight path angle to zero, assuming we burn out in perigee, what's our Argument of Periapsis).
If the rocket takes longer to reach orbit, it'll cover more downrange distance from the launch site, and hence the AoP will be larger (than for a rocket getting to the same orbit faster from the same launch site).

##### Linear Tangent Guidance
Now we can start formulating the math.
This section is meant more as an intuitive explanation than a mathematically rigorous paper, so I will not write any equations here (these can be found below in the Literature section).

As mentioned in the first paragraph on difficulties, there are two forces acting on the vehicle: gravity and thrust.
We model gravity with Newton's inverse square function of position and thrust acceleration with Tsiolkovsky's rocket equation.
As mentioned in the third paragraph, we also need to model *thrust direction* somehow - preferrably with a simple function of time (allowing easy integration) that inputs some parameters (allowing control).
The idea is to have a complete equation of motion with several independent, free variables to solve for, that we can then use to steer the vehicle.
One of those variables we get for free: it's the time-to-MECO, which we can always choose (calculate) differently and get different end results.
A reader with basic understanding of control theory will realize that we need as many control variables as the number of constraints we want to satisfy.

The simplest choice would be a constant function, corresponding to a constant attitude throughout the whole maneuver (KSP players know this well, as it's the primary way of executing on-orbit maneuvers).
Such a function would have only two parameters: pitch and yaw - possibly enough for short duration on-orbit burns, but not for ascent guidance.
In this case adopting a linear function seems like a natural choice.
Linear Tangent Guidance is almost exactly that: it assumes that the *tangent* of pitch (or yaw) is a linear function of time; we could write `pitch=atan(At+B)` (and similarly for yaw).

Parametrizing steering with Linear Tangent function has one large advantage: if we assumed a constant gravity function (that is, gave up the inverse square relation and replace it with an average vector, as if for a flat Earth), the equation of motion becomes directly integrable.
There is a more general version of LTG called Bilinear Tangent Guidance, in which the tangent of each angle is a *ratio* of two linear functions (`pitch=atan((At+B)/(Ct+D))`).
Both these parametrizations are optimal depending on the approximations made (citation needed) and provide closed-form integrals with constant gravity.
Obviously, the flat Earth approximation introduces a potentially large error, which is the more of a problem the longer the burn takes.
Whitcombe (down in the [literature](#literature) section) says however, that there exist "fixes" for that approximation.
Indeed, UPFG also contains an ingenious fix that allows it to benefit from integration simplicity of (a somewhat upgraded) LTG while getting rid of the gross approximation of constant gravity.
This makes it an algorithm of choice even for long-duration maneuvers like Shuttle deorbit burn.

#### Predictor & corrector
* block-by-block explanation:
 * 1 initializes (but we will elaborate on that later as it's poorly explained in the paper)
 * 2 advances state
 * 3, 4, 5 PREDICT final state basing on current guidance
 * 6 calculates pitch&yaw (we will elaborate on that later as it's TBD in the paper)
 * 7, 8 CORRECT guidance so that next prediction is better
---

* iteration:
 * how come it "converges"?
 * initialization problem
* blocks 6, 9
 * 6 determines TVC
 * 9 determines exact MECO (for thrust tail-off) and g-limit mode throttle


### Literature
* D. W. Whitcombe [Present and Advanced Guidance Techniques](http://www.dtic.mil/docs/citations/AD0754923) - "present" refers to 1972, but this is a decent entry-level survey of guidance techniques with many references
* T. J. Brand, D. W. Brown, J. P. Higgins [Unified Powered Flight Guidance](https://ntrs.nasa.gov/search.jsp?R=19740004402) - deep explanation of UPFG with block diagrams and drawings
* F. Teren [Explicit Guidance Equations for Multistage Boost Trajectories](https://ntrs.nasa.gov/search.jsp?R=19660006073) - simpler, earlier algorithm: Powered Explicit Guidance
* [Orbiter Wiki](http://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance) - this article serves as a bridge between tough mathematics of the above paper and understanding of PEG, but does not concern yaw control
