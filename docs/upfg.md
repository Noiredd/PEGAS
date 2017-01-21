## Unified Powered Flight Guidance

### Introduction
There are 3 essential problems with launching a rocket into orbit: determining when should it lift off, how should it travel through the atmosphere, and finally: how to make sure it reaches a precisely defined target orbit.
In this article we will focus on the last part, that is exoatmospheric guidance of a rocket vehicle, explaining one of the most general and well documented algorithms: Unified Powered Flight Guidance.
It was designed specially for the Space Shuttle orbiter, allowing it to carry out tightly constrained missions (most notably the Hubble Space Telescope service missions or ISS construction - both requiring precise rendesvous targetting) while providing a wide range of abort capabilities.

The following assumptions will be made (following the implicit assumptions of the original paper): vehicle is already in motion, high enough that aerodynamic drag can be neglected; only gravity and thrust affect it; there are no coast periods between stages.
The task is to steer the vehicle from its initial state (position, velocity) into a target state (orbit of given inclination, longitude of ascending node, semi-major axis, eccentricity and potentially also argument of periapsis) using only its engines.
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
Unfortunately, the equation we've just written cannot be generally solved this way, which means we cannot explicitly calculate the final state of a thrusting vehicle using a simple formula.
Difficulty number one: how do predict where the vehicle will be in the future, given some thrust vector and gravity?

Second major problem comes from the way we define our initial and target states.
At the beginning the vehicle's state is simplest described using 2 vectors in Cartesian (XYZ) coordinates: position and velocity.
But our target state is an orbit, not a single XYZ point - we would like to use [Keplerian elements](https://en.wikipedia.org/wiki/Orbital_elements) for that.
After all we do not really care *where exactly* will the vehicle be, as long as it's on the right ellipse in the right plane.
And if there is no particular point in space, there's also no particular velocity vector we need to achieve (it changes depending on position on the ellipse).
Difficulty number two: how to formulate the math if we don't have a common coordinate frames for input and output?

By the way, how will our answer even look like?
Let's forget the integration problem for a moment (we can always employ some numeric method to find a solution, or compute it with brute force) and assume we *can* predict the future state of a vehicle.
We know how the gravity will change (always points towards the center of a coordinate frame), and how the thrust magnitude will change, but what about thrust direction vector?
Obviously, we are not interested only in pitch & yaw angles *right here, right now* - we have to somehow express them in a way that allows them to change, and in a way that allows us to integrate this change.
Most logical assumption would be to parametrize them as functions of time (as we are looking for all other solutions as independent functions of time), but then how do we choose those functions?
This is difficulty number three.

Finally, even if we formulated the whole math apparatus to let us predict the future state of a thrusting vehicle in a gravity field, we would only have a set of equations that inputs current vehicle state, gravity parameters and thrust functions (pitch & yaw and magnitude) and outputs a new state in some time from "now".
We need something that works the other way around: inputs current and desired states and outputs pitch & yaw functions.
Final difficulty: how do we reverse those equations, so they yield what we need?

Additionally, it's worth mentioning that we cannot use a half naive, half brute-force approach of precalculating a trajectory on an external (potentially very powerful) computer and then just have a vehicle follow it.
Even the slightest deviations from that ideal path will accumulate on our way, along with errors from the unpredictable elements of the system: imprecise thrust vector control, thrust magnitude variations, turbulent air flow, imperfect gravity model - all can only be approximated, not predicted exactly.
Thus, our guidance system must generate commands on-the-fly, basing on measured vehicle state.
That imposes another difficulty: the system must be simple, robust and work as fast as possible.

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
There's just one thing we cannot define: how will the orbit be rotated within the target plane (or, if we set the flight path angle to zero, assuming we burn out in perigee, what's our argument of periapsis).
If the rocket takes longer to reach orbit, it'll cover more downrange distance from the launch site, and hence the AoP will be larger (than for a rocket getting to the same orbit faster from the same launch site).

##### Linear Tangent Guidance
Now we can start formulating the math.
This section is meant more as an intuitive explanation than a mathematically rigorous paper, so we will not write any equations here (these can be found below in the [literature](#literature) section).

As mentioned in the first paragraph on difficulties, there are two forces acting on the vehicle: gravity and thrust.
We model gravity with Newton's inverse square function of position and thrust acceleration with Tsiolkovsky's rocket equation.
As mentioned in the third paragraph, we also need to model *thrust direction* somehow - preferrably with a simple function of time (allowing easy integration) that inputs some parameters (allowing control).
The idea is to have a complete equation of motion with several independent, free variables to solve for, that we can then use to steer the vehicle.
One of those variables we get for free: it's the time-to-MECO, which we can always choose (calculate) differently and get different end results.
A reader with basic understanding of control theory will realize that we need as many control variables as the number of constraints we want to satisfy.

The simplest choice would be a constant function, corresponding to a constant attitude throughout the whole maneuver (KSP players know this well, as it's the primary way of executing on-orbit maneuvers).
Such a function would have only two parameters: pitch and yaw - possibly enough for short duration on-orbit burns, but not for ascent guidance.
In this case adopting a linear function seems like a natural next choice.
Linear Tangent Guidance is almost exactly that: it assumes that the *tangent* of pitch (or yaw) is a linear function of time; we could write `pitch=atan(At+B)` (and similarly for yaw).

Parametrizing steering with Linear Tangent function has one large advantage: if we assumed a constant gravity function (that is, gave up the inverse square relation and replace it with an average vector, as if for a flat Earth), the equation of motion becomes directly integrable.
There is a more general version of LTG called Bilinear Tangent Guidance, in which the tangent of each angle is a *ratio* of two linear functions (`pitch=atan((At+B)/(Ct+D))`).
Both these parametrizations are optimal depending on the approximations made (citation needed) and provide closed-form integrals with constant gravity.
Obviously, the flat Earth approximation introduces a potentially large error, which is the more of a problem the longer the burn takes.
Whitcombe (see [literature](#literature)) says however, that there exist "fixes" for that approximation.
Indeed, UPFG also contains an ingenious fix that allows it to benefit from integration simplicity of (a somewhat upgraded) LTG while getting rid of the gross approximation of constant gravity.
This makes it an algorithm of choice even for long-duration maneuvers like Shuttle deorbit burn.

##### Predictor & corrector
Integrating forward (current state + thrust vector => future state) is easy thanks to LTG, integrating backwards (current state + future state => thrust vector) is more difficult but also possible.
We however do not have any future state, only constraints on it, which makes the problem harder.
This is why UPFG does not attempt to solve it in a single shot - instead, it uses an iterative predictor-corrector scheme.
That means, for a known current state (position, velocity) some guidance (pitch, yaw, angle rates) is calculated and some final state corresponding to it is *predicted* (it is not necessarily a desired state). Then this final state is *corrected* to meet the constraints, so that the next prediction is closer to desired.
An important notion here is that we're optimizing two things simultaneously: first, we're looking for a future state that will meet the constraints (desired state); second, we're looking for a guidance that will make the vehicle reach this state from the current one.

Some terminology will be introduced now:
* `V` and `R`: current vehicle state (velocity and position),
* `Vd` and `Rd`: desired velocity and position - we want those to meet the constraints set above,
* `Vgrav` and `Rgrav`: changes to vehicle state over the burn trajectory purely due to gravity - purely means: as if the vehicle was coasting,
* `Vgo` and `Rgo`: velocity and position that *need* to be gained due to thrust alone - **required** effect of thrust,
* `Vthrust` and `Rthrust`: velocity and distance that *will be* gained due to thrust alone given some steering constants - **predicted** effect of thrust,
* `Vbias` and `Rbias`: error (*bias*) terms between required (`Vgo`, `Rgo`) and predicted velocity/position change (`Vthrust`, `Rthrust`) - these measure how good our guidance is,
* `Tgo`: time to the end of maneuver - important thing to mention is that in each call, the algorithm assumes the time is zero; hence, `Tgo` should be continuously and stably decreasing, and when it reaches zero thrust should be cut off.

UPFG has several different modes of operation: one for standard ascent to orbit (`Smode=1`), two alternative ascent modes (reference trajectory and Lambert), two abort options and three for on-orbit maneuvers. In here we will focus on the standard ascent mode.

The algorithm is structured in 9 functionally separate "blocks".
The first one (**initialization**) is called exactly once, as it sets up the algorithm with initial values of all variables.
Among those are the `Rd` and `Vd` - exactly how are they chosen is hard to tell (the document does not elaborate on that), and we will get to that later.
Second block (**update**) reads accelerometers and provides the current state `R`, `V` - something we wouldn't have to do in a simplified simulation where all those are directly readable from the simulation memory (like PEGAS or KSP).
It also decreases `Vgo` by velocity gained since the last iteration.

Block 3 (**time-to-go**) calculates `Tgo` for this updated `Vgo` using the rocket equation and knowledge of vehicle parameters in each stage.
In the original paper it is quite complicated, as the implementation contains some pieces specific to the Space Shuttle ascent mode (eg. it accounts for the coast period between SSME cutoff and OMS ignition).
Block 4 (**integrals of thrust**) calculates *thrust integrals* - these describe vehicle capabilities in form of scalar integrals and their *moments*: for example `L` corresponds to remaining delta-v and `S` is "delta-r" (same concept but for distance); `J` will stand for the first moment of velocity, providing some information *on distribution of velocity increment in time* (citation needed).
The equations in this block are arranged as they are to optimize memory usage - many variables are reused in computation of successive ones.

Block 5 (**turning rate**) is where things start to get really interesting.
The main goal here is to calculate the thrust vector and its turning rate and predict its immediate effects.
First, `Rgo` is calculated between current and desired states, taking into account the gravity effect vector (`Rgrav`) as well as previous position error (`Rbias`).
Thrust vector and its turning rate are then explicitly calculated from LTG relationships, using `Vgo`.
Next, change to position and *velocity* due to this thrust vector - `Rthrust` and `Vthrust` - is calculated using integrals from the previous block.
Finally, a check is made how well this trajectory matches the required one: ideally `Rgo` should equal `Rthrust` (which would mean: if this thrust vector and turning rate are maintained, the change of position due to it will exactly cover the difference between the current and desired, countering the gravity pull on the way; the same should be true for velocity).
Difference of those values is stored as an error (`Rbias`, `Vbias`) for the next iteration.

Block 7 (**gravity effects**) integrates the effects of a central gravity field on the thrust trajectory.
The idea is to find displacement and velocity change (`Rgrav`, `Vgrav`) between the current state and the future state (in `Tgo` seconds from "now"), knowing the state change due to thrust alone.
Was there no thrust and the vehicle was freely moving in a gravity field, an exact calculation would be possible - UPFG uses the *Conic State Extrapolation* routine to determine the final state.
However, it is still problematic to incorporate gravity into a powered trajectory without approximating it with a constant vector; a different, more sophisticated but more accurate approximation is made.
In this approach, the actual powered trajectory is replaced with an approximate coast trajectory.
By changing the initial conditions in a special way (using thrust effect integrals `Rthrust`, `Vthrust`) we can generate a coast trajectory that minimizes the average position error between it and the actual powered one.
The modified current state, `Rc1` and `Vc1` is calculated, and then the CSE routine used to integrate trajectory, resulting in final state: `Rc2` and `Vc2`.
Difference between `Rc2` and `Rc1` is then a gravity effect integral `Rgrav` for the approximated powered trajectory; the same follows for velocity.
CSE itself is moderately precise (there exists a more accurate but slower routine, the *Precision State Extrapolation Routine*), but here it works with an approximated trajectory, which naturally results in some error.
For this reason, gravity calculation must be periodically updated to prevent the error from accumulating.

Block 8 (**velocity-to-be-gained**) corrects the desired states (`Vd`, `Rd`), forcing them to meet the constraints.
First the actual cutoff position is predicted (`Rp` - "p" for prediction) simply by summing already obtained vectors: current state (`R` and `V` times `Tgo`), gravity effects, thrust effects.
Then, to obtain a new desired state `Rd`, the magnitude of this vector is corrected to meet the desired altitude - note how this will result in a `Rbias` term change in block 5 in the next iteration.
Finally, an entirely new desired velocity `Vd` is calculated at this position: all remaining constraints are satisfied in this step (normal vector, velocity magnitude, flight path angle).
*Important notice: for some reason, the target plane normal vector* `iY` *in UPFG is oriented opposite to the orbital angular velocity vector.*
`Vgo` is then updated as a difference between this desired velocity and the current one, keeping in mind that the gravity will also affect the vehicle's motion (`Vgrav`) and propagating a remaining velocity error (`Vbias`) for the next iteration.

##### Convergence
If we wrote all the equations into simplified relations using only symbols defined in the terminology section (skipping the intermediate variables used in the actual formulation), we'd find that everything is a function of everything else.
Block 5 uses position terms (current, desired, gravity effects integral, error term) to generate guidance, which it then uses to predict thrust effects integrals (both velocity and position terms); these are finally used to calculate new errors.
Block 7 updates gravity integrals estimation over the new trajectory.
Block 8 uses position terms (current, new gravity integral, new thrust effect prediction) and constraints to generate a new desired state.
Then it uses velocity terms (current, freshly generated desired one, gravity integral, error term) to calculate a new velocity to go that will be used in the next call to block 5.

When the algorithm is first called, the error between predicted and desired states can be very large and thus the resulting steering constants are not very reliable.
The original document refers to this phase as a *prethrust mode* - each call (=*iteration*) the errors should decrease and the guidance approach a good one.
After several iterations the predicted state should closely match the desired - when that happens we say that the algorithm has *converged*, and the vehicle can safely start following the calculated guidance.
From then on the algorithm is in *active guidance mode* - each next call only corrects for any errors (eg. from gravity calculation).

Last remaining problem: if everything is a function of everything and the algorithm somehow "converges" on the "right" guidance and predictions... where do we start from?
Block 5 uses desired position `Rd` which is not known until block 8, and without `Rd` there is no `Vd` without which there can be no `Vgo` which is used as early as block 2.
Unfortunately the original paper does not provide any information on how to determine the initial value of `Rd`.
PEGAS implementation contains a custom logic to generate an initial guess on this value.
It follows from an (empirically confirmed) assumption that if this guess is reasonable enough, UPFG will be able to correct for the guess error and converge on a true value.
The reasoning behind the guess is the fact that the vehicle is already in motion in some direction - so it is only possible that the cutoff position will be somewhere that way.
Just how far ahead (in terms of downrange distance) is a matter of vehicle performance: for example, low-thrust upper stages can make very long burn arcs.
PEGAS generates the initial guess by rotating the initial position vector in the direction of motion by an arbitrary angle and correcting the resulting vector so that it meets plane and altitude constraints.
Then `Vd` is calculated for this position following the equation in block 8; `Vgo` is initialized very simply as `Vd` minus initial `V`.
It turns out that these values result in UPFG convergence within 2-3 iterations (for a reference Space Shuttle mission, convergence criterion: change in `Tgo` between iterations less than 1%).

### Epilogue
Two of the blocks were not mentioned in the article: 6 and 9.
Skipped on purpose, they are not used in guidance and could be omitted altogether in a very simplified simulation; the original document labels them both as *To Be Determined*.
Block 6 (**steering commands**) would generate thrust vector *commands* - that is, convert the thrust direction vector `iF` and thrust turning rate vector into pitch and yaw angles and rates (perhaps also engine gimbal commands?).
This block is implemented in PEGAS using simple vector transformations (pitch is the angle between `iF` and local UP vector `unit(R)`, yaw is the angle between planar component of `iF` and local EAST); angle rates are not calculated.
Block 9 (**throttle commands**) would generate throttle setting and engine on/off commands.
Standard ascent mode uses either a constant-thrust mode (throttle being set to max) or a constant-acceleration mode, where the throttle is gradually reduced in order to enforce a G-limit on the vehicle.
Other modes (eg. ascent to reference trajectory) might use active throttling to provide an additional degree of control freedom.
Additionally, this block would issue an engine-off command basing on its characteristics, probably to allow a precisely timed shutdown accounting for thrust tail-off and valve response times.

The above should walk the reader through the UPFG algorithm and provide a general understanding of the concept.
For a more in-depth understanding, *iterating* over the original document, preferrably with a pencil and paper, is recommended.


### Literature
* D. W. Whitcombe [Present and Advanced Guidance Techniques](http://www.dtic.mil/docs/citations/AD0754923) - "present" refers to 1972, but this is a decent entry-level survey of guidance techniques with many references
* T. J. Brand, D. W. Brown, J. P. Higgins [Unified Powered Flight Guidance](https://ntrs.nasa.gov/search.jsp?R=19740004402) - original formulation of UPFG with equations, block diagrams and drawings
* F. Teren [Explicit Guidance Equations for Multistage Boost Trajectories](https://ntrs.nasa.gov/search.jsp?R=19660006073) - simpler, earlier algorithm: Powered Explicit Guidance
* Orbiter Wiki [Powered Explicit Guidance](http://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance) - this article serves as a bridge between tough mathematics of the above paper and understanding of PEG, but does not concern yaw control
